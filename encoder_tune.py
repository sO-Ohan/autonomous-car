#!/usr/bin/env python3
"""
Live AS5600 magnet-tuning dashboard.

Run it, then move each magnet with your hands and watch the gauges. The number
that matters is AGC: the AS5600 turns its internal amplifier gain up when the
magnetic field is weak and down when it is strong, so AGC is an inverse proxy
for field strength at the die. Pegged at 128 (3.3 V max) means the sensor has
run out of gain -- the magnet is too far. Pegged at 0 means too close.
Aim for the middle of the range, around 64.

    python3 encoder_tune.py [/dev/ttyUSB0]
"""

import sys
import time
from collections import deque

import serial
try:                                    # PyQt6 if present, PyQt5 otherwise --
    from PyQt6.QtCore import (Qt, QThread, pyqtSignal, QTimer,   # noqa: F401
                              QRectF, QPointF)
    from PyQt6.QtGui import (QPainter, QColor, QPen, QBrush, QFont,
                             QPainterPath, QLinearGradient)
    from PyQt6.QtWidgets import (QApplication, QWidget, QHBoxLayout,
                                 QVBoxLayout, QLabel, QSizePolicy)
except ImportError:                     # the scoped-enum syntax below works on both
    from PyQt5.QtCore import (Qt, QThread, pyqtSignal, QTimer,   # noqa: F401
                              QRectF, QPointF)
    from PyQt5.QtGui import (QPainter, QColor, QPen, QBrush, QFont,
                             QPainterPath, QLinearGradient)
    from PyQt5.QtWidgets import (QApplication, QWidget, QHBoxLayout,
                                 QVBoxLayout, QLabel, QSizePolicy)

# ----------------------------------------------------------------- constants
AGC_MAX = 128           # AS5600 range on a 3.3 V supply (it is 0-255 at 5 V)
AGC_TARGET = 64
BAND_GOOD = (40, 88)
BAND_OK = (24, 112)
HISTORY_SECONDS = 20.0

# ----------------------------------------------------------------- palette
BG = QColor("#0E1116")
PANEL = QColor("#151A21")
STROKE = QColor("#262E39")
TEXT = QColor("#C9D4E0")
DIM = QColor("#6B7A8C")
GOOD = QColor("#3FB950")
WARN = QColor("#D29922")
BAD = QColor("#F0616D")
ACCENT = QColor("#58A6FF")


def agc_color(agc):
    if agc is None or agc < 0:
        return DIM
    if agc <= 0 or agc >= AGC_MAX:
        return BAD
    if BAND_GOOD[0] <= agc <= BAND_GOOD[1]:
        return GOOD
    if BAND_OK[0] <= agc <= BAND_OK[1]:
        return WARN
    return BAD


def agc_verdict(agc, ml, mh, md):
    if agc is None or agc < 0:
        return "NO DATA", DIM
    if not md:
        return "NO MAGNET DETECTED", BAD
    if agc >= AGC_MAX or ml:
        return "TOO FAR  ·  MOVE CLOSER", BAD
    if agc <= 0 or mh:
        return "TOO CLOSE  ·  BACK OFF", BAD
    if BAND_GOOD[0] <= agc <= BAND_GOOD[1]:
        return "GOOD", GOOD
    if agc > BAND_GOOD[1]:
        return "A BIT FAR  ·  CLOSER", WARN
    return "A BIT CLOSE  ·  BACK OFF", WARN


# ----------------------------------------------------------------- serial
class Reader(QThread):
    sample = pyqtSignal(dict)
    status = pyqtSignal(str)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = True

    def run(self):
        try:
            s = serial.Serial(self.port, 115200, timeout=0.3)
        except Exception as e:
            self.status.emit(f"cannot open {self.port}: {e}")
            return

        # hardware reset, wait out the boot diagnostic, then ask for telemetry
        self.status.emit("resetting board...")
        s.setDTR(False)
        s.setRTS(True)
        time.sleep(0.15)
        s.setRTS(False)
        time.sleep(0.05)
        s.reset_input_buffer()
        time.sleep(5.0)
        s.reset_input_buffer()
        s.write(b"t\n")
        self.status.emit(f"streaming from {self.port}")

        buf = b""
        while self.running:
            try:
                chunk = s.read(512)
            except Exception as e:
                self.status.emit(f"serial error: {e}")
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip().decode("utf-8", "replace")
                if not line.startswith("T,"):
                    continue
                parts = line.split(",")
                if len(parts) != 9:
                    continue
                try:
                    v = [int(x) for x in parts[1:]]
                except ValueError:
                    continue
                self.sample.emit({
                    "t": time.time(),
                    "L": {"ang": v[0], "agc": v[1], "mag": v[2], "stat": v[3]},
                    "R": {"ang": v[4], "agc": v[5], "mag": v[6], "stat": v[7]},
                })
        try:
            s.write(b"t\n")
            s.close()
        except Exception:
            pass

    def stop(self):
        self.running = False


# ----------------------------------------------------------------- widgets
class Gauge(QWidget):
    """270-degree arc for AGC, with the good band drawn into the scale."""

    def __init__(self):
        super().__init__()
        self.agc = None
        self.best = None
        self.setMinimumHeight(210)
        self.setSizePolicy(QSizePolicy.Policy.Expanding,
                           QSizePolicy.Policy.Expanding)

    def set_value(self, agc, best):
        self.agc, self.best = agc, best
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        side = min(w, h * 1.35)
        r = side / 2 - 26
        cx, cy = w / 2, h / 2 + r * 0.22
        box = QRectF(cx - r, cy - r, 2 * r, 2 * r)

        start, span = 225.0, -270.0            # degrees, Qt convention

        def frac(v):
            return max(0.0, min(1.0, v / AGC_MAX))

        # track
        p.setPen(QPen(STROKE, 14, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawArc(box, int(start * 16), int(span * 16))

        # good band, drawn as part of the scale so the target is a place
        g0, g1 = frac(BAND_GOOD[0]), frac(BAND_GOOD[1])
        band = QColor(GOOD)
        band.setAlpha(70)
        p.setPen(QPen(band, 14, Qt.PenStyle.SolidLine, Qt.PenCapStyle.FlatCap))
        p.drawArc(box, int((start + span * g0) * 16), int((span * (g1 - g0)) * 16))

        # value
        if self.agc is not None and self.agc >= 0:
            c = agc_color(self.agc)
            p.setPen(QPen(c, 14, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            p.drawArc(box, int(start * 16), int(span * frac(self.agc) * 16))

        # best-so-far tick: tells you whether you are improving
        if self.best is not None:
            import math
            a = math.radians(start + span * frac(self.best))
            x, y = cx + math.cos(a), cy - math.sin(a)
            p.setPen(QPen(ACCENT, 2))
            p.drawLine(QPointF(cx + (r - 12) * math.cos(a), cy - (r - 12) * math.sin(a)),
                       QPointF(cx + (r + 12) * math.cos(a), cy - (r + 12) * math.sin(a)))

        # readout
        f = QFont("monospace")
        f.setPointSize(40)
        f.setBold(True)
        p.setFont(f)
        p.setPen(agc_color(self.agc))
        txt = "--" if self.agc is None or self.agc < 0 else str(self.agc)
        p.drawText(QRectF(0, cy - r * 0.75, w, r * 0.8),
                   Qt.AlignmentFlag.AlignCenter, txt)

        f.setPointSize(9)
        f.setBold(False)
        p.setFont(f)
        p.setPen(DIM)
        p.drawText(QRectF(0, cy - r * 0.06, w, 20),
                   Qt.AlignmentFlag.AlignCenter, f"AGC   target {AGC_TARGET}")

        # end labels
        p.drawText(QRectF(cx - r - 18, cy + r * 0.62, 40, 16),
                   Qt.AlignmentFlag.AlignCenter, "0")
        p.drawText(QRectF(cx + r - 22, cy + r * 0.62, 40, 16),
                   Qt.AlignmentFlag.AlignCenter, str(AGC_MAX))
        p.end()


class Strip(QWidget):
    """AGC against time, so a slow hand movement shows a direction."""

    def __init__(self):
        super().__init__()
        self.hist = deque()
        self.setMinimumHeight(120)

    def push(self, t, agc):
        if agc is not None and agc >= 0:
            self.hist.append((t, agc))
        cutoff = time.time() - HISTORY_SECONDS
        while self.hist and self.hist[0][0] < cutoff:
            self.hist.popleft()
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        pad = 8

        p.fillRect(self.rect(), PANEL)

        def ypx(v):
            return h - pad - (v / AGC_MAX) * (h - 2 * pad)

        # good band
        band = QColor(GOOD)
        band.setAlpha(28)
        p.fillRect(QRectF(0, ypx(BAND_GOOD[1]), w,
                          ypx(BAND_GOOD[0]) - ypx(BAND_GOOD[1])), band)

        # target line
        p.setPen(QPen(QColor(63, 185, 80, 140), 1, Qt.PenStyle.DashLine))
        p.drawLine(QPointF(0, ypx(AGC_TARGET)), QPointF(w, ypx(AGC_TARGET)))

        # ceiling -- the line you are currently stuck on
        p.setPen(QPen(QColor(240, 97, 109, 110), 1, Qt.PenStyle.DashLine))
        p.drawLine(QPointF(0, ypx(AGC_MAX)), QPointF(w, ypx(AGC_MAX)))

        if len(self.hist) >= 2:
            now = time.time()
            path = QPainterPath()
            fill = QPainterPath()
            for i, (t, v) in enumerate(self.hist):
                x = w - ((now - t) / HISTORY_SECONDS) * w
                y = ypx(v)
                if i == 0:
                    path.moveTo(x, y)
                    fill.moveTo(x, h)
                    fill.lineTo(x, y)
                else:
                    path.lineTo(x, y)
                    fill.lineTo(x, y)
            fill.lineTo(w, h)
            fill.closeSubpath()

            c = agc_color(self.hist[-1][1])
            grad = QLinearGradient(0, 0, 0, h)
            g = QColor(c)
            g.setAlpha(90)
            grad.setColorAt(0.0, g)
            g2 = QColor(c)
            g2.setAlpha(0)
            grad.setColorAt(1.0, g2)
            p.fillPath(fill, QBrush(grad))

            p.setPen(QPen(c, 2))
            p.drawPath(path)

        p.setPen(QPen(STROKE, 1))
        p.drawRect(QRectF(0, 0, w - 1, h - 1))
        f = QFont("monospace")
        f.setPointSize(8)
        p.setFont(f)
        p.setPen(DIM)
        p.drawText(QRectF(6, 4, 200, 14), Qt.AlignmentFlag.AlignLeft,
                   f"AGC · last {int(HISTORY_SECONDS)} s")
        p.end()


class Dial(QWidget):
    """Shaft angle plus a turn counter -- proves the magnet rotates with it."""

    def __init__(self):
        super().__init__()
        self.ang = None
        self.turns = 0.0
        self.setFixedHeight(150)

    def set_value(self, ang, turns):
        self.ang, self.turns = ang, turns
        self.update()

    def paintEvent(self, _):
        import math
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        r = min(w, h) / 2 - 16
        cx, cy = w / 2, h / 2

        p.setPen(QPen(STROKE, 2))
        p.setBrush(QBrush(PANEL))
        p.drawEllipse(QPointF(cx, cy), r, r)

        for i in range(12):
            a = math.radians(i * 30)
            p.setPen(QPen(STROKE, 1))
            p.drawLine(QPointF(cx + (r - 6) * math.cos(a), cy - (r - 6) * math.sin(a)),
                       QPointF(cx + r * math.cos(a), cy - r * math.sin(a)))

        if self.ang is not None and self.ang >= 0:
            deg = self.ang * 360.0 / 4096.0
            a = math.radians(90 - deg)
            p.setPen(QPen(ACCENT, 3, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            p.drawLine(QPointF(cx, cy),
                       QPointF(cx + (r - 10) * math.cos(a), cy - (r - 10) * math.sin(a)))
            p.setBrush(QBrush(ACCENT))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(QPointF(cx, cy), 4, 4)

            f = QFont("monospace")
            f.setPointSize(9)
            p.setFont(f)
            p.setPen(DIM)
            p.drawText(QRectF(0, h - 18, w, 16), Qt.AlignmentFlag.AlignCenter,
                       f"{deg:6.1f}°   raw {self.ang:4d}   turns {self.turns:+.2f}")
        p.end()


class Led(QWidget):
    def __init__(self, name, good_when_set):
        super().__init__()
        self.name = name
        self.good_when_set = good_when_set
        self.on = None
        self.setFixedSize(74, 34)

    def set_on(self, on):
        self.on = on
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        if self.on is None:
            c = DIM
        elif self.on == self.good_when_set:
            c = GOOD
        else:
            c = BAD
        p.setBrush(QBrush(c if self.on else PANEL))
        p.setPen(QPen(c, 1.5))
        p.drawEllipse(QPointF(12, 17), 6, 6)
        f = QFont("monospace")
        f.setPointSize(9)
        p.setFont(f)
        p.setPen(TEXT if self.on else DIM)
        p.drawText(QRectF(24, 8, 50, 18), Qt.AlignmentFlag.AlignLeft, self.name)
        p.end()


class Panel(QWidget):
    """One encoder: gauge, verdict, history, status bits, shaft dial."""

    def __init__(self, title, subtitle):
        super().__init__()
        self.prev_ang = None
        self.turns = 0.0
        self.best = None

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 16, 18, 16)
        lay.setSpacing(10)

        head = QLabel(title)
        head.setStyleSheet(
            f"color:{TEXT.name()};font-family:monospace;font-size:15px;"
            "font-weight:700;letter-spacing:2px;")
        sub = QLabel(subtitle)
        sub.setStyleSheet(f"color:{DIM.name()};font-family:monospace;font-size:11px;")
        lay.addWidget(head)
        lay.addWidget(sub)

        self.gauge = Gauge()
        lay.addWidget(self.gauge)

        self.verdict = QLabel("waiting")
        self.verdict.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.verdict.setStyleSheet(
            f"color:{DIM.name()};font-family:monospace;font-size:13px;"
            "font-weight:700;letter-spacing:1px;padding:8px;"
            f"border:1px solid {STROKE.name()};border-radius:4px;")
        lay.addWidget(self.verdict)

        self.strip = Strip()
        lay.addWidget(self.strip)

        leds = QHBoxLayout()
        leds.setSpacing(2)
        self.led_md = Led("MD", True)      # magnet detected -- want it set
        self.led_ml = Led("ML", False)     # too weak -- want it clear
        self.led_mh = Led("MH", False)     # too strong -- want it clear
        for x in (self.led_md, self.led_ml, self.led_mh):
            leds.addWidget(x)
        leds.addStretch()
        self.magl = QLabel("MAGNITUDE --")
        self.magl.setStyleSheet(
            f"color:{DIM.name()};font-family:monospace;font-size:11px;")
        leds.addWidget(self.magl)
        lay.addLayout(leds)

        self.dial = Dial()
        lay.addWidget(self.dial)

        self.setStyleSheet(
            f"Panel{{background:{PANEL.name()};border:1px solid {STROKE.name()};"
            "border-radius:8px;}")

    def update_sample(self, t, d):
        agc, stat, ang, mag = d["agc"], d["stat"], d["ang"], d["mag"]

        if agc is not None and 0 <= agc:
            err = abs(agc - AGC_TARGET)
            if self.best is None or err < abs(self.best - AGC_TARGET):
                self.best = agc

        self.gauge.set_value(agc, self.best)
        self.strip.push(t, agc)

        md = ml = mh = None
        if stat is not None and stat >= 0:
            md = bool(stat & 0x20)
            ml = bool(stat & 0x10)
            mh = bool(stat & 0x08)
        self.led_md.set_on(md)
        self.led_ml.set_on(ml)
        self.led_mh.set_on(mh)
        self.magl.setText(f"MAGNITUDE {mag if mag is not None and mag >= 0 else '--'}")

        text, c = agc_verdict(agc, ml, mh, md)
        if self.best is not None and agc is not None and agc >= 0:
            text += f"      best {self.best}"
        self.verdict.setText(text)
        self.verdict.setStyleSheet(
            f"color:{c.name()};font-family:monospace;font-size:13px;"
            "font-weight:700;letter-spacing:1px;padding:8px;"
            f"border:1px solid {c.name()};border-radius:4px;")

        if ang is not None and ang >= 0:
            if self.prev_ang is not None:
                dd = ang - self.prev_ang
                if dd > 2048:
                    dd -= 4096
                if dd < -2048:
                    dd += 4096
                self.turns += dd / 4096.0
            self.prev_ang = ang
        self.dial.set_value(ang, self.turns)


class Main(QWidget):
    def __init__(self, port):
        super().__init__()
        self.setWindowTitle("AS5600 magnet tuning")
        self.resize(940, 760)
        self.setStyleSheet(f"background:{BG.name()};")

        root = QVBoxLayout(self)
        root.setContentsMargins(18, 16, 18, 14)
        root.setSpacing(12)

        title = QLabel("MAGNET AIR-GAP TUNING")
        title.setStyleSheet(
            f"color:{TEXT.name()};font-family:monospace;font-size:17px;"
            "font-weight:700;letter-spacing:3px;")
        root.addWidget(title)

        hint = QLabel(
            "Move each magnet closer to its sensor until AGC comes down off 128 "
            "and settles near 64.\nThe blue tick on the arc is the best reading "
            "so far, so you can tell whether you are improving.")
        hint.setStyleSheet(
            f"color:{DIM.name()};font-family:monospace;font-size:11px;line-height:150%;")
        root.addWidget(hint)

        cols = QHBoxLayout()
        cols.setSpacing(14)
        self.left = Panel("LEFT", "mux channel 6 · mask 0x40 · GPIO 32/33")
        self.right = Panel("RIGHT", "mux channel 3 · mask 0x08 · GPIO 25/26")
        cols.addWidget(self.left)
        cols.addWidget(self.right)
        root.addLayout(cols, 1)

        self.status = QLabel("starting")
        self.status.setStyleSheet(
            f"color:{DIM.name()};font-family:monospace;font-size:11px;")
        root.addWidget(self.status)

        self.count = 0
        self.t0 = time.time()
        self.reader = Reader(port)
        self.reader.sample.connect(self.on_sample)
        self.reader.status.connect(self.status.setText)
        self.reader.start()

        self.tick = QTimer(self)
        self.tick.timeout.connect(self.refresh_rate)
        self.tick.start(1000)

    def on_sample(self, s):
        self.count += 1
        self.left.update_sample(s["t"], s["L"])
        self.right.update_sample(s["t"], s["R"])

    def refresh_rate(self):
        dt = time.time() - self.t0
        if dt > 0 and self.count:
            self.status.setText(
                f"{self.count} samples · {self.count / dt:4.1f} Hz · "
                f"AGC 0-128 (3.3 V scale) · high AGC = weak field = too far")

    def closeEvent(self, e):
        self.reader.stop()
        self.reader.wait(1500)
        e.accept()


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    app = QApplication(sys.argv)
    win = Main(port)
    win.show()
    sys.exit(app.exec() if hasattr(app, "exec") else app.exec_())
