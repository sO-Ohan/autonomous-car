# Ethernet link: console and firmware updates over the W5500

Everything the USB cable used to carry — the console, telemetry, and firmware
updates — now also goes over Ethernet:

```
ESP32-C3  --SPI-->  W5500  --RJ45-->  switch  --RJ45-->  Bullet M2  ))) 2.4 GHz ))) laptop
```

USB is not removed. It stays as the recovery path, and it is the only way to
install the first Ethernet-capable image. See [Bootstrap](#bootstrap-the-one-usb-flash-you-still-have-to-do).

Status as of 2026-08-31: **working on the car, end to end.** Flashed once over
USB, then a second image was pushed over Ethernet and booted from the spare
partition. Measured numbers in [What is verified](#what-is-verified-and-what-is-not).

---

## Why

The USB CDC link re-enumerates whenever the motors draw current. That is a
reproducible hardware fact (see `SETUP.md`, and the `usb-drops-when-motors-run`
note), and it drops the console at exactly the moment you care about — while
the car is moving. The W5500 does not care what the motors are doing, and it
reaches the laptop through the Bullet instead of a tether.

A second, quieter reason: with the cable out, the car can actually drive.

---

## Bootstrap: the one USB flash you still have to do

You cannot push this image over the network, because the firmware currently on
the board has no Ethernet stack — only the `Y` probe. There is nothing on the
car to receive an OTA yet. So exactly once, over USB:

```fish
cd /home/so1/drive-harness-diag/c3-webgui
pio run -e esp32-c3-supermini -t upload
```

Flashing the C3 is fiddly and unchanged by any of this — any DTR toggle resets
the board, so it needs the BOOT-hold plus held-open-fd dance (`SETUP.md`).
That is the last time you need to do it.

The partition table already has two app slots (`ota_0` / `ota_1`, 1.25 MB
each), so **no repartitioning is needed** and this flash does not wipe
anything. Confirmed against `default.csv`.

After it boots, watch the console — **without resetting it**, see
[the DTR warning](#do-not-reset-this-board-by-toggling-dtrrts):

```fish
stty -F /dev/ttyACM0 115200 raw -echo -hupcl clocal
cat /dev/ttyACM0
```

What this car actually printed on its first Ethernet boot:

```
eth: MAC 3C:0F:02:A4:D8:2F, trying DHCP...
eth: W5500 up, STATIC (no DHCP answer) 192.168.1.77, link UP
eth: console on tcp/3333, firmware push on tcp/3232
```

That was the original DHCP-first firmware. Since 2026-09-15 the address is
**always static** and the first line reads `eth: MAC ..., static 192.168.1.77`
(see [Addressing](#addressing-always-static) below).

From then on the USB cable is optional. Verify with `ping 192.168.1.77`.

---

## Daily use

### Console + telemetry

```fish
nc 192.168.1.77 3333          # or any line-oriented client
```

Everything the USB console did, identically — the same commands, the same
`ODO,` / `FB,` / `T,` / `MAG,` / `IMU,` lines. Nothing about the protocol
changed, which is the point.

The Python tooling follows with one environment variable, no code changes:

```fish
set -x CARBOT_PORT tcp://192.168.1.77
python3 motor_tui.py            # and encoder_tune.py, odom.py, ...
```

`carbot/driver.py` accepts all of these and picks the transport itself:

| Port string          | Transport            |
|----------------------|----------------------|
| `/dev/ttyACM0`       | USB serial           |
| `tcp://192.168.1.77` | TCP, port 3333       |
| `192.168.1.77:3333`  | TCP, explicit port   |
| `192.168.1.77`       | TCP, port 3333       |

### Firmware updates

```fish
cd /home/so1/drive-harness-diag/c3-webgui
pio run -e ota -t upload
```

That builds and pushes over the radio link. Change the target address in
`platformio.ini` under `[env:ota]`, or call the pusher directly:

```fish
python3 ota_push.py 192.168.1.77 .pio/build/esp32-c3-supermini/firmware.bin
```

The board verifies an MD5 of the whole image **before** it commits, so a
transfer damaged by a marginal wireless link is rejected rather than flashed.
On failure the old image keeps running and the tool exits non-zero.

### `C` — link status

New console command:

```
C
# eth: started, static, ip 192.168.1.77, link UP
# eth: MAC 02:..., console CONNECTED, ports 3333/3232
# eth: tx 184320 bytes in 152 packets (1212.6 B/pkt), 3 OTA(s) served
# eth: running image from partition 'app1'
```

The bytes-per-packet figure is the one to watch — see below.

---

## How the communication was optimised

Three things, and the second is the one that matters.

**1. USB writes are gated on the port actually being open.**
`HWCDC::write()` blocks for up to 100 ms per call when USB is unplugged — core
2.0.17, `HWCDC.cpp:440`, *"waits 1ms per sending data attempt, in case CDC is
unplugged"*. This firmware emits ~110 telemetry lines/s. With the cable out and
no gate, that alone would stall `loop()` into uselessness. Every USB write now
checks `if (Serial)`, which is `HWCDC::operator bool()` — the real connected
state. **This bug would have bitten the moment you unplugged the cable**, and
it is the single most important line in the change.

**2. Telemetry is batched into full segments.**
Un-batched, every `Serial.printf()` becomes its own TCP segment: ~110 packets/s
carrying ~40 bytes each, every one paying a 40-byte TCP/IP header plus a full
802.11 exchange. On a 2.4 GHz link, airtime is spent per *packet*, not per
byte — so that shape of traffic is roughly the worst thing you can do to it.

Writes now land in a 1460-byte (one MTU) buffer flushed on a 20 ms timer, or
early at 1200 bytes. Result: **≤50 full packets/s instead of ~110 ragged
ones**, usually far fewer, carrying the same data. `C` reports the achieved
bytes-per-packet; if that number is near 40 something is flushing too eagerly.

20 ms of buffering is invisible on a console and is nowhere near any control
deadline — control lives in `controlTask()` and never comes through here.

**3. Commands go the other way with `TCP_NODELAY`.**
Inbound is latency-sensitive and tiny: a `V` command sitting in a Nagle buffer
is a wheel that starts late. So the host side disables Nagle. Batching outbound
and not batching inbound is deliberate, not an inconsistency.

---

## Safety behaviour

Three things the network path must never get wrong, all handled:

- **A dropped console stops the car.** When the TCP client goes away, control
  is disengaged immediately. The existing `CMD_TIMEOUT` watchdog would also
  catch it, but that is up to 400 ms of travel first.
- **An OTA stops the motors before writing a single byte to flash.** An update
  must not be able to leave the car driving, and the reboot at the end would
  otherwise resume from whatever state the controller was in.
- **A new console connection displaces a stale one.** After a laptop sleeps the
  old socket can sit half-open for minutes; without this you would be locked
  out of your own car.

There is **no authentication** on either port. That is a deliberate choice, not
an oversight: it would be security theatre while the Bullet itself still ships
on factory `ubnt`/`ubnt` with a root shell. Fix the radio's credentials — that
is the real door. See [Open items](#open-items).

---

## What is verified, and what is not

Being precise about this, because the difference matters.

**Verified:**
- The firmware compiles and links. Flash went 69.9% → **73.9%** (968100 of
  1310720 bytes), RAM 13.8%. Comfortable headroom.
- The partition table has OTA slots, so no repartition is required.
- `carbot/driver.py` transport selection, all four port forms, by test.
- The OTA wire protocol end to end, against a loopback mock of the firmware
  side: a good image is accepted, and an image corrupted by one flipped bit is
  **rejected on MD5** rather than flashed. That test also caught a real bug in
  `ota_push.py` (it ignored a `host:port` argument), now fixed.

**Verified on the car, 2026-08-31:**
- **SPI is sound at 14 MHz.** This was the main worry and it is settled: the
  W5500 initialises at the Ethernet library's clock, not just at the 8 MHz the
  `Y` probe uses. Link comes up 100 Mbps full duplex.
- Ping to the car: **0.3 ms**, no loss.
- Console over TCP works, and the batching does what it was built to do:
  **1170 bytes delivered in 1 packet**, versus the ~30 separate segments the
  same burst would have cost unbatched.
- **OTA over Ethernet works.** 993.1 kB in **2.9 s (345 kB/s)**, MD5 accepted,
  and the board came back running from partition `app1` having previously run
  from `app0` — so the image really was written to the spare slot and booted.
- The eFuse ETH MAC (`3C:0F:02:A4:D8:2F`) is correctly distinct from the WiFi
  MAC (`...:2C`), so bringing the radio up with `A 1` cannot collide.

**Known, and expected:**
- <a id="addressing-always-static"></a>**The address is always static:
  192.168.1.77/24, gateway .1.** The firmware used to try DHCP first and only
  fall back to .77 when nobody answered. On 2026-09-15 a second router on the
  bench switch (`172.19.23.1`) answered, the car came up at `172.19.23.64`, and
  .77 was dead from both the laptop and `mt10` while the link itself was fine.
  DHCP was removed: every host on the SLAMBOT network is static (`NETWORK.md`),
  and the car's address must not depend on which router happens to be plugged
  in. `C` reports `static`.
- If you ever move the car to a segment where SPI is marginal, `ethBegin()`
  will say `NO HARDWARE at 14 MHz` while `Y` still says `PRESENT`. That means
  the leads are too long or too noisy for the library clock: shorten them, or
  drop the clock in `.pio/libdeps/*/Ethernet/src/utility/w5100.h` line 21.

---

## Troubleshooting

### Do not reset this board by toggling DTR/RTS

Learned the hard way during bring-up, and worth writing down because the
symptom looks like a dead board.

Trying to force a clean reset from software — `setDTR`/`setRTS` from pyserial,
the usual esptool-style pulse — puts this C3 into **ROM download mode** and
leaves it there:

```
rst:0x15 (USB_UART_CHIP_RESET),boot:0x5 (DOWNLOAD(USB/UART0/1))
waiting for download
```

After that first banner it is completely silent, which reads exactly like a
crashed or bricked board. It is neither. `esptool.py run` did **not** get it
out, because every electrical reset available goes through the same two lines
that put it there.

**The fix is physical: unplug the USB cable and plug it back in**, or press
reset/EN. Nothing else works.

So when you want to watch it boot, do not reset it — just open the port
without touching the control lines:

```fish
stty -F /dev/ttyACM0 115200 raw -echo -hupcl clocal
cat /dev/ttyACM0
```

`-hupcl` is the important flag: it stops DTR being dropped when the port is
opened or closed. This is the same hazard already recorded for flashing; it
applies to *reading* too.

### Table

| Symptom | Cause | Fix |
|---|---|---|
| Board silent, `waiting for download` seen once | DTR/RTS was toggled | Unplug and replug USB — software reset will not clear it |
| Boot says `NO HARDWARE at 14 MHz`, `Y` says `PRESENT` | SPI marginal at the library's clock | Shorten leads, or lower `SPI_ETHERNET_SETTINGS` |
| `link DOWN` | cable / switch / Bullet port | Check the switch LEDs first |
| Board reachable, console silent | another console holds the socket | Second connection gets `# busy`; drop the first |
| `ota_push.py` cannot connect / .77 dead but link UP | old DHCP-first firmware took an address from another router | Plug USB, run `C`, read the real IP; flash current firmware (always static) |
| OTA reports `ERR md5 mismatch` | corrupted transfer | Old image still runs; just retry |
| Car reachable but drives late | Nagle | Confirm the host sets `TCP_NODELAY` |

Recovery is always the same: plug the USB cable back in. The USB console
survives everything here, and reads prefer USB so a cable always wins a tie.

---

## Open items

1. **Change the Bullet's credentials.** It is on factory `ubnt`/`ubnt` and
   lands as `uid=0`. Once it is transmitting from the car, anyone in RF range
   owns the radio — and through it, both open ports on the car.
2. **Confirm the Bullet's role.** `/etc/board.info` carries
   `board.cap.station_only`; check it before deciding which end of the link
   the Bullet serves.
3. **Wire the W5500 `INT` pin** if you ever want the ESP-IDF `esp_eth` driver
   (real lwIP, mDNS, `car.local`) instead of the W5500's own hardware TCP/IP
   stack. Not needed for anything above, and it would also require moving to
   Arduino core 3.x — which is a much larger change than this one was.
4. **Consider raising the SPI clock.** 14 MHz is proven on this wiring and
   gives 345 kB/s OTA, which is already fine. The W5500 is rated well past
   30 MHz, so if a future image gets much larger there is headroom to take.
5. **The USB console still costs something even when idle.** Writes are gated
   on the port being open, so an unplugged cable is free — but a *plugged in
   but unread* port is not. Unplug it when running on the radio.

---

## Implementation notes

For anyone reading the diff.

The change adds ~370 lines to `c3-webgui/src/main.cpp` and modifies **six**
existing lines. That ratio is the design: `LinkStream` is a `Stream` that fans
writes to both USB and TCP and reads from whichever side is talking, and a
single `#define Serial Link` re-points all ~1200 existing call sites at it. No
`Serial.printf()` in the firmware was touched.

Two things load-bearing enough to state plainly:

- **`controlTask()` must never print.** The TX buffer has no mutex, which is
  correct only because every `Serial.*` call in this firmware is made from
  `loop()`. If you ever print from the control task, add a mutex first.
- Arduino's `EthernetServer` is **abstract on ESP32** — the core's `Server`
  base declares `virtual void begin(uint16_t) = 0` and the Ethernet library
  only ever defined the no-argument `begin()`. The `EthServer` subclass exists
  solely to close that gap. This is why the stock library appears not to work
  on ESP32; it does, with six lines.

The previous image is at `src/main.cpp.bak-preeth`.

Files touched:

| File | Change |
|---|---|
| `c3-webgui/src/main.cpp` | Ethernet transport, OTA server, `C` command |
| `c3-webgui/platformio.ini` | `Ethernet` dep, `[env:ota]` upload target |
| `c3-webgui/ota_push.py` | new — host-side firmware pusher |
| `carbot/driver.py` | TCP transport, `CARBOT_PORT` |
