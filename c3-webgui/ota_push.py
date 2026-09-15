#!/usr/bin/env python3
"""Push a firmware image to the car over Ethernet (W5500 -> switch -> Bullet).

    ./ota_push.py 192.168.1.77 .pio/build/esp32-c3-supermini/firmware.bin

or, the way you will normally use it:

    pio run -e ota -t upload

The protocol is deliberately trivial, and matches netOtaService() in
src/main.cpp:

    -> "OTA <bytes> <md5hex>\\n"   then <bytes> raw bytes
    <- "OK <bytes>\\n"  or  "ERR <reason>\\n"

The board verifies the MD5 itself before it commits the image, so a truncated
or corrupted transfer over a marginal wireless link is rejected rather than
flashed. On success the board reboots into the new image and this script
exits 0; the connection dropping right after "OK" is expected, not an error.
"""
import hashlib
import os
import socket
import sys
import time

OTA_PORT = 3232
CONNECT_TIMEOUT = 5.0
TRANSFER_TIMEOUT = 30.0
CHUNK = 4096


def push(host, path, port=OTA_PORT):
    with open(path, "rb") as fh:
        image = fh.read()
    if not image:
        raise SystemExit("error: %s is empty" % path)

    md5 = hashlib.md5(image).hexdigest()
    size = len(image)
    print("firmware : %s" % path)
    print("size     : %d bytes (%.1f kB)" % (size, size / 1024.0))
    print("md5      : %s" % md5)
    print("target   : %s:%d" % (host, port))

    started = time.time()
    sock = socket.create_connection((host, port), CONNECT_TIMEOUT)
    sock.settimeout(TRANSFER_TIMEOUT)
    try:
        sock.sendall(b"OTA %d %s\n" % (size, md5.encode()))

        sent = 0
        for off in range(0, size, CHUNK):
            sock.sendall(image[off:off + CHUNK])
            sent += len(image[off:off + CHUNK])
            pct = 100 * sent // size
            print("\ruploading: %3d%%  %6.1f kB" % (pct, sent / 1024.0),
                  end="", flush=True)
        print()

        reply = b""
        while b"\n" not in reply:
            chunk = sock.recv(256)
            if not chunk:
                break
            reply += chunk
    finally:
        sock.close()

    reply = reply.decode(errors="replace").strip()
    elapsed = time.time() - started

    if reply.startswith("OK"):
        print("board    : %s" % reply)
        print("done     : %.1f s (%.0f kB/s), board is rebooting"
              % (elapsed, size / 1024.0 / elapsed))
        return 0

    print("board    : %s" % (reply or "<no reply>"), file=sys.stderr)
    print("FAILED. The old image is still running -- the board only commits "
          "after the MD5 checks out.", file=sys.stderr)
    return 1


def main(argv):
    if len(argv) != 3:
        print(__doc__)
        return 2
    host, path = argv[1], argv[2]
    port = OTA_PORT
    if host.startswith("tcp://"):
        host = host[len("tcp://"):]
    if ":" in host:
        host, _, portstr = host.rpartition(":")
        port = int(portstr)
    if not os.path.exists(path):
        print("error: no such file: %s" % path, file=sys.stderr)
        return 2
    try:
        return push(host, path, port)
    except socket.timeout:
        print("error: timed out talking to %s:%d" % (host, port),
              file=sys.stderr)
        return 1
    except OSError as exc:
        print("error: cannot reach %s:%d (%s)" % (host, port, exc),
              file=sys.stderr)
        print("       check: ping the car, then `C` on the console for its IP",
              file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
