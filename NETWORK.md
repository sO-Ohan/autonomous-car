# Network: the SLAMBOT radio link

How the laptop reaches everything on the robot over the Bullet M2, and how to
see the robot's ROS 2 data without installing ROS on the laptop.

`ETHERNET.md` covers the car's own side of this link (ESP32-C3 → W5500 →
switch). This file covers the radio, the robot PC and the laptop.

Set up and verified: 2026-09-15.

> Passwords are deliberately **not** in this repository (it is public).
> Get them from the team.

---

## Topology

```
                         robot                                    ground
 ┌──────────────────────────────────────────────┐
 │ ESP32-C3 + W5500  192.168.1.77 ──┐           │
 │                                  ├─ switch ──┼─ Bullet M2 ))) 2.4 GHz ))) laptop
 │ robot PC mt10     192.168.1.69 ──┘           │  192.168.1.1   "SLAMBOT"   192.168.1.11 (wifi)
 │  └ foxglove_bridge  ws://:8765               │  access point               192.168.1.10 (wired)
 └──────────────────────────────────────────────┘
```

The Bullet is an **access point bridged to its Ethernet port**, and the laptop
joins it as an ordinary wifi client. That puts every device on one flat layer-2
network, `192.168.1.0/24`, so there is no NAT, no routing and nothing to
forward. SSH, the car's TCP console and ROS 2 all work as if cabled.

### Why the laptop joins the Bullet, and not the other way round

The obvious alternative, Bullet as a client of a laptop hotspot, was rejected.
A wifi *client* cannot bridge the devices behind it onto the hotspot's network
without 4-address (WDS) mode. NetworkManager hotspots do not offer WDS. Without
it the robot ends up behind NAT or a pseudo-bridge, and ROS 2 discovery (which
is multicast) stops working. With the Bullet as the access point, bridging
comes for free.

## Addresses

| Device | Address | Notes |
|---|---|---|
| Bullet M2 (OpenWrt) | `192.168.1.1` | web UI + SSH, user `root` |
| Laptop, wired | `192.168.1.10` | bench only, via PoE injector LAN port |
| Laptop, wifi | `192.168.1.11` | NetworkManager profile `SLAMBOT` |
| Robot PC `mt10` | `192.168.1.69` | Ubuntu 22.04, ROS 2 Humble, NM profile `rover` |
| Car (ESP32-C3/W5500) | `192.168.1.77` | console tcp/3333, OTA tcp/3232, see `ETHERNET.md` |

All static. Nothing on this network has an internet uplink, so nothing on it
should use `192.168.1.1` as a default gateway (see [Robot PC](#robot-pc-mt10)).

---

## Bullet M2

| Item | Value |
|---|---|
| Hardware | Ubiquiti Bullet M2, rev **XW** (AR9342), 8 MB flash |
| Firmware | OpenWrt 25.12.0, profile `ubnt_bullet-m-xw` |
| Mode | access point, `wifi-iface` on `lan` (bridged with `eth0`) |
| SSID / security | `SLAMBOT`, WPA2-PSK |
| Channel | 6 (2437 MHz), **HT20** |
| Country | `BD` |
| TX power | maximum (no `txpower` option set) |
| `disassoc_low_ack` | `0` |
| `multicast_to_unicast_all` | `1` |

Applied with:

```sh
uci set wireless.radio0.channel=6
uci set wireless.radio0.htmode=HT20
uci set wireless.radio0.country=BD
uci -q delete wireless.radio0.txpower
uci set wireless.default_radio0.disassoc_low_ack=0
uci set wireless.default_radio0.multicast_to_unicast_all=1
uci commit wireless && wifi reload
```

### Why these settings

- **HT20, not HT40.** A 20 MHz channel roughly halves peak throughput (72 vs
  150 Mbit/s) but concentrates the same transmit power into half the
  bandwidth, and avoids overlapping neighbouring 2.4 GHz networks. At range
  that means a steadier link, which matters more than peak speed here.
- **`disassoc_low_ack=0`.** By default hostapd kicks a client that misses
  ACKs. At the edge of range that turns a weak link into no link. Now it stays
  associated and rate control drops to a slower, more robust rate instead.
- **`multicast_to_unicast_all=1`.** Wifi sends multicast at the lowest basic
  rate with no retries. Converting it to per-client unicast makes ROS 2
  discovery and mDNS traffic both faster and reliable.

### Transmit power: what "maximum" actually is

OpenWrt reports **19 dBm**. That is the AR9342 chip's own limit from its
factory calibration, and nothing higher can be selected. It is not a
regulatory cap: changing the country to `US` (30 dBm limit) was tested and
changed nothing, so the country was left at `BD`.

The Bullet has an external power amplifier after the chip, which OpenWrt does
not account for. Ubiquiti rates the unit at about 28 dBm at the connector.
This was **not measured** here; checking it needs an RF power meter.

| Point | dBm | Watts |
|---|---|---|
| Chip, as OpenWrt reports | 19 | 0.08 W |
| Antenna connector (Ubiquiti rating) | ~28 | ~0.63 W |
| Radiated, with 8 dBi antenna | ~36 | ~4 W EIRP |
| Radiated, with 12 dBi antenna | ~40 | ~10 W EIRP |

Every +3 dB of antenna gain doubles the radiated power. The Bullet can never
put 4 W into the cable, but it gets to ~4 W EIRP with an 8–9 dBi antenna.

### Flashing and recovery

Stock airOS ↔ OpenWrt via TFTP recovery is documented in
`~/bulletm2-backup/FLASHING-GUIDE.md` on the laptop, next to the full flash
dumps (`mtd0`–`mtd5`). Those are not in this repo: they are device-specific
and contain the calibration data.

**The reset button wipes the OpenWrt config**, including the SLAMBOT network
and the root password. After a reset the Bullet comes back at `192.168.1.1`
with no password and wifi disabled. Re-apply the `uci` block above.

---

## Robot PC (`mt10`)

### Foxglove bridge

`ros-humble-foxglove-bridge` is installed and runs as a systemd service that
starts on boot and restarts if it dies:

```ini
# /etc/systemd/system/foxglove-bridge.service
[Unit]
Description=Foxglove bridge (ROS 2 -> ws://:8765)
After=network-online.target
Wants=network-online.target

[Service]
User=mt10
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 address:=0.0.0.0"
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

```sh
systemctl status foxglove-bridge          # is it up
journalctl -u foxglove-bridge -f          # what it is advertising
sudo systemctl restart foxglove-bridge    # after changing ROS_DOMAIN_ID etc.
```

It must run with the **same `ROS_DOMAIN_ID`** as the robot's nodes. Both are
currently unset (domain 0). If the nodes move to another domain, add
`Environment=ROS_DOMAIN_ID=<n>` to the service.

**Why a bridge, and not ROS 2 over the radio directly.** Raw DDS over wifi
means multicast discovery plus one UDP stream per topic per subscriber, and it
behaves badly on a lossy 2.4 GHz link. The bridge keeps DDS local to the robot
PC and sends only the topics the viewer subscribes to, over a single TCP
WebSocket. That is one connection, flow-controlled, and cheap on airtime.

### Default route

The wired profile `rover` (`eno2`, `192.168.1.69/24`) had `192.168.1.1` as
its gateway. The Bullet has no uplink, so that route blackholed all internet
traffic even when the robot PC's own wifi was connected. It is now
never-default:

```sh
sudo nmcli con modify rover ipv4.never-default yes ipv6.never-default yes
sudo nmcli dev reapply eno2
```

Internet, when there is any, goes out over `wlo1`.

### Installing packages when the robot PC has no internet

Borrow the laptop's connection through SSH. No network changes are needed:

```sh
# from the laptop
ssh -R 1080 mt10@192.168.1.69
# then on mt10
sudo apt-get -o Acquire::http::Proxy=socks5h://127.0.0.1:1080 update
sudo apt-get -o Acquire::http::Proxy=socks5h://127.0.0.1:1080 install <pkg>
```

---

## Laptop

### Wifi profile

```sh
nmcli con modify SLAMBOT \
  ipv4.method manual ipv4.addresses 192.168.1.11/24 ipv4.never-default yes \
  ipv6.method disabled \
  802-11-wireless.powersave 2 \
  connection.autoconnect-priority 10

nmcli con up SLAMBOT
```

- **Static address.** Nothing depends on a DHCP server being present.
- **`never-default`.** Joining SLAMBOT never steals the laptop's internet
  route, so wired or other internet keeps working.
- **`powersave 2` (off).** Wifi power saving parks the radio between beacons
  and adds tens of milliseconds, in bursts, to every reply. That is fatal for
  teleop.
- **Priority 10.** When SLAMBOT is in range and the laptop is picking a
  network, it picks the robot.

Joining SLAMBOT uses the laptop's wifi card, so it leaves any other wifi
network (e.g. campus wifi).

### Viewer: Lichtblick

[Lichtblick](https://github.com/lichtblick-suite/lichtblick) is the
open-source continuation of Foxglove Studio: same panels, no account.
Installed unpacked (no root needed) in
`~/Applications/lichtblick-1.29.1-linux-x64`, with an app-menu entry
**Lichtblick (SLAMBOT)**.

1. Connect the laptop to SLAMBOT.
2. Open Lichtblick → **Open connection** → **Foxglove WebSocket**.
3. URL: `ws://192.168.1.69:8765`.

Every ROS 2 topic on the robot PC is then available: plots, raw messages,
images, TF, 3D, maps, `/rosout` logs. Launch from a terminal with
`--no-sandbox` (the bundled Chrome sandbox helper is not setuid).

---

## What is verified

Measured 2026-09-15 with the laptop on the bench, wifi only (`ping -I wlan0`):

| Check | Result |
|---|---|
| Laptop signal at the Bullet | −41 to −46 dBm |
| PHY rate | 72.2 Mbit/s (MCS 7, short GI, the HT20 ceiling) |
| Laptop → Bullet ping | 1.7 ms avg |
| Laptop → robot PC ping, 30 packets | 2.4 ms avg, 5.6 ms max, **0 % loss** |
| Foxglove WebSocket handshake over wifi | `101 Switching Protocols` |
| Bridge advertising topics | `/rosout`, `/parameter_events` (no other nodes running) |

**Not verified:** range in the field, throughput under load (camera streams),
and the ~28 dBm output figure. The first ping after joining showed ~78 ms
while the link settled. Measure after a few seconds, not immediately.

## Troubleshooting

| Symptom | Check |
|---|---|
| Can't SSH to the Bullet | Did someone press reset? Then it is `192.168.1.1` with no password and no wifi. Cable in and re-apply the config |
| No SLAMBOT in the wifi list | `ssh root@192.168.1.1 iwinfo` over the cable, and check `wireless.radio0.disabled` |
| Joined SLAMBOT, can't reach `192.168.1.69` | laptop has `192.168.1.11`? (`ip -br a show wlan0`). Robot PC cabled into the switch? |
| Lichtblick connects but shows no topics | robot nodes not running, or on a different `ROS_DOMAIN_ID` from the bridge |
| Laggy, bursty latency | laptop wifi power saving came back: `iw dev wlan0 get power_save` |
| Robot PC has no internet | expected on this network. Use the SSH SOCKS trick above |

## Open items

- **Second Bullet.** There are two units. Only one (Ethernet MAC
  `AC:8B:A9:C2:76:0A`) has been found on the network and configured. The
  other has not been seen yet.
- **Field range test** with the real antennas, and a decision on HT20 vs HT40
  based on measured throughput at distance.
- **Credentials.** The Bullet root password, WPA2 key and robot PC login are
  shared, simple, and used in more than one place. Change them before any
  public event.
