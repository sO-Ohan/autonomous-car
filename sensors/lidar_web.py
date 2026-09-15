#!/usr/bin/env python3
"""Car sensor web GUI: RPLIDAR live plot + control, IMU, link status. Stdlib only.

    python3 lidar_web.py                       # http://<this box>:8080
    python3 lidar_web.py --http-port 8080 --lidar-port /dev/ttyUSB1 --baud 256000

The lidar is read through the Slamtec SDK's `ultra_simple` (built in ~/rplidar_sdk),
which is the only path this firmware streams on -- the legacy 5-byte SCAN yields
nothing, see lidar_probe.py. Its stdout lines look like

    S  theta: 4.37 Dist: 02224.00 Q: 47      <- "S" marks the first point of a sweep

We run it under `stdbuf -oL` so lines arrive as they are printed, cut the stream
into sweeps on the S flag, and keep the latest sweep for /api/scan. SIGINT makes
ultra_simple stop the scan and park the motor, so stop == SIGINT.

The IMU (WitMotion, 11-byte 0x55 frames, see imu_probe.py) is read in its own
thread when present. The ESP32 motor board is only pinged; driving stays in
motor_tui.py where the safety watchdog lives.
"""
import argparse, glob, json, os, re, signal, struct, subprocess, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

HERE = os.path.dirname(os.path.abspath(__file__))
SDK_BIN = os.path.expanduser("~/rplidar_sdk/output/Linux/Release/ultra_simple")
CAR_IP = "192.168.1.77"
LINE = re.compile(r"^(S| )\s+theta:\s*([\d.]+)\s+Dist:\s*([\d.]+)\s+Q:\s*(\d+)")

# ----------------------------------------------------------------------------- lidar
class Lidar:
    def __init__(self, port, baud):
        self.cfg = {"port": port, "baud": baud, "max_range_m": 12.0, "min_quality": 0}
        self.proc = None
        self.lock = threading.Lock()
        self.sweep = []              # latest complete sweep: [[deg, mm, q], ...]
        self.partial = []
        self.info = {}               # serial / firmware / hw / health from the banner
        self.stats = {"sweeps": 0, "hz": 0.0, "pts_per_s": 0, "last_sweep_t": 0.0}
        self.log = []                # last stderr / banner lines for the UI
        self.error = ""
        self._pts_window = []

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self):
        with self.lock:
            if self.running():
                return "already running"
            if not os.path.exists(SDK_BIN):
                self.error = f"SDK binary missing: {SDK_BIN}"
                return self.error
            if not os.path.exists(self.cfg["port"]):
                self.error = f"port {self.cfg['port']} not present"
                return self.error
            self.error, self.info, self.log = "", {}, []
            self.proc = subprocess.Popen(
                ["stdbuf", "-oL", SDK_BIN, "--channel", "--serial",
                 self.cfg["port"], str(self.cfg["baud"])],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            threading.Thread(target=self._reader, args=(self.proc,), daemon=True).start()
            return "started"

    def stop(self):
        with self.lock:
            p, self.proc = self.proc, None
        if p and p.poll() is None:
            p.send_signal(signal.SIGINT)          # ultra_simple: stop scan, motor off
            try:
                p.wait(3)
            except subprocess.TimeoutExpired:
                p.kill()
        return "stopped"

    def _reader(self, p):
        t_sweep = time.time()
        for raw in p.stdout:
            m = LINE.match(raw)
            if not m:
                line = raw.strip()
                if line:
                    self.log = (self.log + [line])[-12:]
                    for key, pat in (("serial", r"S/N:\s*(\w+)"), ("firmware", r"Firmware Ver:\s*([\d.]+)"),
                                     ("hardware", r"Hardware Rev:\s*(\d+)"), ("health", r"health status\s*:\s*(\d+)")):
                        mm = re.search(pat, line)
                        if mm:
                            self.info[key] = mm.group(1)
                    if "Error" in line:
                        self.error = line
                continue
            flag, deg, mm_, q = m.groups()
            if flag == "S" and self.partial:
                now = time.time()
                dt = now - t_sweep
                t_sweep = now
                self._pts_window.append((now, len(self.partial)))
                self._pts_window = [(t, n) for t, n in self._pts_window if now - t < 2.0]
                span = now - self._pts_window[0][0] if len(self._pts_window) > 1 else 1.0
                with self.lock:
                    self.sweep = self.partial
                    self.stats = {"sweeps": self.stats["sweeps"] + 1,
                                  "hz": round(1.0 / dt, 1) if dt > 0 else 0.0,
                                  "pts_per_s": int(sum(n for _, n in self._pts_window) / max(span, 0.1)),
                                  "last_sweep_t": now}
                self.partial = []
            self.partial.append([float(deg), float(mm_), int(q)])
        with self.lock:
            if self.proc is p:
                self.proc = None
            if p.returncode not in (0, None, -2) and not self.error:
                self.error = f"ultra_simple exited {p.returncode}"

    def snapshot(self):
        with self.lock:
            return {"sweep": self.sweep, "stats": self.stats, "running": self.running(),
                    "info": self.info, "error": self.error, "cfg": self.cfg, "log": self.log}

# ------------------------------------------------------------------------------- imu
class Imu:
    """WitMotion 0x55 frames at 115200: 0x51 accel, 0x52 gyro, 0x53 angle (see imu_probe.py)."""
    def __init__(self, port):
        self.port, self.data, self.ok, self.err = port, {}, False, ""
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            import serial
        except ImportError:
            self.err = "pyserial missing"; return
        while True:
            if not (self.port and os.path.exists(self.port)):
                self.ok = False; time.sleep(2); continue
            try:
                with serial.Serial(self.port, 115200, timeout=1) as s:
                    buf = b""
                    while True:
                        buf += s.read(64)
                        if not buf:
                            self.ok = False; continue
                        i = buf.find(b"\x55")
                        while i >= 0 and len(buf) - i >= 11:
                            f = buf[i:i + 11]
                            if sum(f[:10]) & 0xFF == f[10]:
                                a, b, c, _ = struct.unpack("<hhhh", f[2:10])
                                if f[1] == 0x51: self.data.update(ax=a / 2048, ay=b / 2048, az=c / 2048)
                                elif f[1] == 0x52: self.data.update(gx=a / 16.384, gy=b / 16.384, gz=c / 16.384)
                                elif f[1] == 0x53: self.data.update(roll=a / 182.04, pitch=b / 182.04, yaw=c / 182.04)
                                self.data["t"] = time.time(); self.ok = True
                                i += 11
                            else:
                                i += 1
                            j = buf.find(b"\x55", i)
                            i = j
                        buf = buf[i:] if i >= 0 else b""
            except Exception as e:                # unplugged mid-stream: back off and retry
                self.err, self.ok = str(e), False; time.sleep(2)

# ------------------------------------------------------------------------------ system
def ports():
    out = []
    for link in sorted(glob.glob("/dev/serial/by-id/*")):
        dev = os.path.realpath(link)
        name = os.path.basename(link)
        guess = "RPLIDAR (CP2102)" if "CP2102" in name else "IMU (CH340)" if "1a86" in name else ""
        out.append({"dev": dev, "id": name, "guess": guess})
    return out

class CarPing(threading.Thread):
    def __init__(self, ip):
        super().__init__(daemon=True); self.ip, self.up, self.ms, self.t = ip, False, None, 0; self.start()
    def run(self):
        while True:
            r = subprocess.run(["ping", "-c1", "-W1", self.ip], capture_output=True, text=True)
            m = re.search(r"time=([\d.]+)", r.stdout)
            self.up, self.ms, self.t = r.returncode == 0, float(m.group(1)) if m else None, time.time()
            time.sleep(3)

# ------------------------------------------------------------------------------- http
class Handler(BaseHTTPRequestHandler):
    lidar: Lidar; imu: Imu; car: CarPing

    def log_message(self, *a):  # quiet
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers(); self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path); q = parse_qs(u.query)
        if u.path == "/":
            body = PAGE.encode()
            self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)
        elif u.path == "/api/scan":
            s = self.lidar.snapshot()
            self._json(s)
        elif u.path == "/api/status":
            self._json({"lidar": {k: v for k, v in self.lidar.snapshot().items() if k != "sweep"},
                        "imu": {"port": self.imu.port, "ok": self.imu.ok, "err": self.imu.err, "data": self.imu.data},
                        "car": {"ip": self.car.ip, "up": self.car.up, "ms": self.car.ms},
                        "ports": ports(), "sdk": os.path.exists(SDK_BIN), "host": os.uname().nodename,
                        "time": time.time()})
        elif u.path == "/api/start":
            self._json({"result": self.lidar.start(), "error": self.lidar.error})
        elif u.path == "/api/stop":
            self._json({"result": self.lidar.stop()})
        elif u.path == "/api/config":
            was = self.lidar.running()
            if was: self.lidar.stop()
            c = self.lidar.cfg
            if "port" in q: c["port"] = q["port"][0]
            if "baud" in q: c["baud"] = int(q["baud"][0])
            if "max_range_m" in q: c["max_range_m"] = float(q["max_range_m"][0])
            if "min_quality" in q: c["min_quality"] = int(q["min_quality"][0])
            if "imu_port" in q: self.imu.port = q["imu_port"][0]
            if was: self.lidar.start()
            self._json({"cfg": c, "imu_port": self.imu.port})
        elif u.path == "/favicon.ico":
            self.send_response(204); self.end_headers()
        else:
            self._json({"error": "not found"}, 404)

# ------------------------------------------------------------------------------- page
PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Car Lidar</title>
<style>
:root{--bg:#0d1117;--panel:#161b22;--line:#30363d;--fg:#e6edf3;--dim:#8b949e;--acc:#3fb950;--warn:#d29922;--bad:#f85149;--pt:#58a6ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,sans-serif}
header{display:flex;align-items:center;gap:14px;padding:10px 16px;border-bottom:1px solid var(--line);flex-wrap:wrap}
header h1{font-size:16px;margin:0 8px 0 0;font-weight:600}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:var(--dim);margin-right:6px;vertical-align:middle}
.dot.ok{background:var(--acc)}.dot.bad{background:var(--bad)}.dot.warn{background:var(--warn)}
button{background:#21262d;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:6px 14px;font:inherit;cursor:pointer}
button:hover{border-color:#8b949e}button.primary{background:#238636;border-color:#2ea043}button.stop{background:#8b1e1e;border-color:#b62324}
button:disabled{opacity:.45;cursor:default}
main{display:grid;grid-template-columns:minmax(0,1fr) 340px;gap:12px;padding:12px}
@media(max-width:900px){main{grid-template-columns:1fr}}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:12px}
.panel h2{font-size:12px;text-transform:uppercase;letter-spacing:.08em;color:var(--dim);margin:0 0 8px}
#plot{width:100%;aspect-ratio:1;display:block;background:#0a0e14;border-radius:6px;touch-action:none}
.kv{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;font-variant-numeric:tabular-nums}
.kv span:first-child{color:var(--dim)}.kv b{font-weight:500;word-break:break-all}
label{display:block;color:var(--dim);font-size:12px;margin-top:8px}
input,select{width:100%;background:#0d1117;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:6px 8px;font:inherit;margin-top:3px}
.row{display:flex;gap:8px;align-items:end}.row>*{flex:1}
pre{margin:0;font:11px/1.4 ui-monospace,monospace;color:var(--dim);white-space:pre-wrap;max-height:130px;overflow:auto}
.stack{display:flex;flex-direction:column;gap:12px}
.err{color:var(--bad)}
small{color:var(--dim)}
</style></head><body>
<header>
 <h1>Car Lidar</h1>
 <span><i class="dot" id="d-lidar"></i><span id="s-lidar">lidar</span></span>
 <span><i class="dot" id="d-imu"></i><span id="s-imu">imu</span></span>
 <span><i class="dot" id="d-car"></i><span id="s-car">motor board</span></span>
 <span style="flex:1"></span>
 <button class="primary" id="btn-start" onclick="api('start')">Start scan</button>
 <button class="stop" id="btn-stop" onclick="api('stop')">Stop</button>
</header>
<main>
 <div class="panel">
  <canvas id="plot"></canvas>
  <div class="kv" style="grid-template-columns:repeat(4,auto 1fr);margin-top:8px">
   <span>sweep rate</span><b id="k-hz">–</b><span>points/s</span><b id="k-pps">–</b>
   <span>points</span><b id="k-n">–</b><span>nearest</span><b id="k-near">–</b>
  </div>
  <small>Scroll / pinch to zoom · drag to pan · double-click to reset · 0° is the lidar's connector side; red marker = nearest return</small>
 </div>
 <div class="stack">
  <div class="panel"><h2>Lidar</h2>
   <div class="kv" id="lidar-info"></div>
   <div class="err" id="lidar-err"></div>
  </div>
  <div class="panel"><h2>Config</h2>
   <label>Lidar port<select id="c-port"></select></label>
   <div class="row">
    <label>Baud<select id="c-baud"><option>115200</option><option>256000</option><option>460800</option><option>1000000</option></select></label>
    <label>Max range (m)<input id="c-range" type="number" min="1" max="40" step="0.5"></label>
    <label>Min quality<input id="c-q" type="number" min="0" max="63"></label>
   </div>
   <label>IMU port<select id="c-imu"></select></label>
   <button style="margin-top:10px;width:100%" onclick="saveCfg()">Apply (restarts scan if running)</button>
  </div>
  <div class="panel"><h2>IMU</h2><div class="kv" id="imu-kv"><span>—</span><b></b></div></div>
  <div class="panel"><h2>System</h2><div class="kv" id="sys-kv"></div><pre id="log" style="margin-top:8px"></pre></div>
 </div>
</main>
<script>
const $=id=>document.getElementById(id);
let cfg={max_range_m:12,min_quality:0}, view={zoom:1,x:0,y:0}, last=null, portsSeen="";
const cv=$('plot'), ctx=cv.getContext('2d');
function fit(){const r=cv.getBoundingClientRect(), s=devicePixelRatio||1; if(cv.width!==r.width*s){cv.width=r.width*s;cv.height=r.width*s;} }
function draw(sw){
 fit(); const W=cv.width, c=W/2, R=(W/2-16*(devicePixelRatio||1))*view.zoom, ox=c+view.x, oy=c+view.y, maxR=cfg.max_range_m*1000;
 ctx.clearRect(0,0,W,W); ctx.save(); ctx.lineWidth=1;
 ctx.strokeStyle='#1f2630'; ctx.fillStyle='#4b5563'; ctx.font=`${11*(devicePixelRatio||1)}px system-ui`;
 const step=cfg.max_range_m>10?2:1;
 for(let m=step;m<=cfg.max_range_m+0.01;m+=step){const r=R*m/cfg.max_range_m; ctx.beginPath();ctx.arc(ox,oy,r,0,7);ctx.stroke(); ctx.fillText(m+' m',ox+4,oy-r-3);}
 for(let a=0;a<360;a+=30){const t=(a-90)*Math.PI/180; ctx.beginPath();ctx.moveTo(ox,oy);ctx.lineTo(ox+R*Math.cos(t),oy+R*Math.sin(t));ctx.stroke();
   ctx.fillText(a+'°',ox+(R+10)*Math.cos(t)-8,oy+(R+10)*Math.sin(t)+4);}
 ctx.strokeStyle='#3fb950';ctx.beginPath();ctx.moveTo(ox,oy);ctx.lineTo(ox,oy-R*0.08);ctx.stroke();
 let near=null;
 if(sw&&sw.length){ ctx.fillStyle='#58a6ff';
  for(const [deg,mm,q] of sw){ if(mm<=0||q<cfg.min_quality||mm>maxR) continue; if(!near||mm<near[1]) near=[deg,mm,q];
   const t=(deg-90)*Math.PI/180, r=R*mm/maxR; ctx.fillRect(ox+r*Math.cos(t)-1.2,oy+r*Math.sin(t)-1.2,2.4,2.4);}
  if(near){const t=(near[0]-90)*Math.PI/180,r=R*near[1]/maxR; ctx.strokeStyle='#f85149';ctx.lineWidth=2;ctx.beginPath();ctx.arc(ox+r*Math.cos(t),oy+r*Math.sin(t),6,0,7);ctx.stroke();}
 }
 ctx.restore(); return near;
}
async function poll(){
 try{const s=await (await fetch('/api/scan')).json(); last=s; cfg={...cfg,...s.cfg};
  const near=draw(s.sweep);
  $('k-hz').textContent=s.stats.hz?s.stats.hz+' Hz':'–'; $('k-pps').textContent=s.stats.pts_per_s||'–';
  $('k-n').textContent=s.sweep.length||'–'; $('k-near').textContent=near?(near[1]/1000).toFixed(2)+' m @ '+near[0].toFixed(0)+'°':'–';
  const stale=s.running&&s.stats.last_sweep_t&&(Date.now()/1000-s.stats.last_sweep_t>2);
  setDot('lidar',s.running?(stale?'warn':'ok'):'', s.running?(stale?'lidar: no data':'lidar: scanning'):'lidar: stopped');
  $('btn-start').disabled=s.running; $('btn-stop').disabled=!s.running;
  $('lidar-err').textContent=s.error||'';
 }catch(e){setDot('lidar','bad','server unreachable');}
 setTimeout(poll, 120);
}
function setDot(k,cls,txt){$('d-'+k).className='dot '+cls; $('s-'+k).textContent=txt;}
function kv(el,pairs){el.innerHTML=pairs.map(([k,v])=>`<span>${k}</span><b>${v??'–'}</b>`).join('');}
async function status(){
 try{const s=await (await fetch('/api/status')).json(); const L=s.lidar, I=s.imu, C=s.car;
  kv($('lidar-info'),[['serial',L.info.serial],['firmware',L.info.firmware],['hardware rev',L.info.hardware],
    ['health',L.info.health==null?null:['GOOD','WARNING','ERROR'][L.info.health]],['sweeps',L.stats.sweeps],['port',L.cfg.port+' @ '+L.cfg.baud]]);
  const d=I.data, f=(x,u='')=>x==null?'–':x.toFixed(1)+u;
  kv($('imu-kv'),[['roll',f(d.roll,'°')],['pitch',f(d.pitch,'°')],['yaw',f(d.yaw,'°')],['accel g',d.ax==null?'–':`${d.ax.toFixed(2)} ${d.ay.toFixed(2)} ${d.az.toFixed(2)}`],
    ['gyro °/s',d.gx==null?'–':`${d.gx.toFixed(0)} ${d.gy.toFixed(0)} ${d.gz.toFixed(0)}`],['port',I.port]]);
  setDot('imu',I.ok?'ok':'bad',I.ok?'imu: live':'imu: '+(I.err||'no data'));
  setDot('car',C.up?'ok':'bad',C.up?`motor board ${C.ip} ${C.ms} ms`:`motor board ${C.ip} unreachable`);
  kv($('sys-kv'),[['host',s.host],['sdk',s.sdk?'built':'MISSING'],...s.ports.map(p=>[p.dev,p.guess||p.id.slice(0,28)])]);
  $('log').textContent=L.log.join('\n');
  const key=JSON.stringify(s.ports);
  if(key!==portsSeen){portsSeen=key; for(const [id,cur] of [['c-port',L.cfg.port],['c-imu',I.port]]){const sel=$(id); sel.innerHTML='';
     for(const p of s.ports){const o=document.createElement('option');o.value=p.dev;o.textContent=p.dev+(p.guess?'  — '+p.guess:'');sel.append(o);}
     if(![...sel.options].some(o=>o.value===cur)){const o=document.createElement('option');o.value=cur;o.textContent=cur+'  (absent)';sel.append(o);} sel.value=cur;}
   $('c-baud').value=L.cfg.baud; $('c-range').value=L.cfg.max_range_m; $('c-q').value=L.cfg.min_quality;}
 }catch(e){}
 setTimeout(status,1500);
}
async function api(a){await fetch('/api/'+a);}
async function saveCfg(){
 const p=new URLSearchParams({port:$('c-port').value,baud:$('c-baud').value,max_range_m:$('c-range').value,min_quality:$('c-q').value,imu_port:$('c-imu').value});
 await fetch('/api/config?'+p); portsSeen=''; }
cv.addEventListener('wheel',e=>{e.preventDefault(); view.zoom=Math.min(20,Math.max(0.3,view.zoom*(e.deltaY<0?1.15:1/1.15))); draw(last&&last.sweep);},{passive:false});
let drag=null; cv.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];cv.setPointerCapture(e.pointerId)});
cv.addEventListener('pointermove',e=>{if(!drag)return; const s=devicePixelRatio||1; view.x+=(e.clientX-drag[0])*s; view.y+=(e.clientY-drag[1])*s; drag=[e.clientX,e.clientY]; draw(last&&last.sweep);});
cv.addEventListener('pointerup',()=>drag=null); cv.addEventListener('dblclick',()=>{view={zoom:1,x:0,y:0};draw(last&&last.sweep);});
addEventListener('resize',()=>draw(last&&last.sweep));
poll(); status();
</script></body></html>"""

# -------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--lidar-port", default="/dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=256000)
    ap.add_argument("--imu-port", default="/dev/ttyUSB0")
    ap.add_argument("--autostart", action="store_true", help="start scanning immediately")
    a = ap.parse_args()
    Handler.lidar = Lidar(a.lidar_port, a.baud)
    Handler.imu = Imu(a.imu_port)
    Handler.car = CarPing(CAR_IP)
    if a.autostart:
        Handler.lidar.start()
    srv = ThreadingHTTPServer(("0.0.0.0", a.http_port), Handler)
    print(f"serving on http://0.0.0.0:{a.http_port}  (lidar {a.lidar_port} @ {a.baud}, imu {a.imu_port})")
    try:
        srv.serve_forever()
    finally:
        Handler.lidar.stop()

if __name__ == "__main__":
    main()
