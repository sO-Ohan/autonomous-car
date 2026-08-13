#!/usr/bin/env python3
"""
One connection helper for both transports.

Pass a serial device (`/dev/ttyUSB0`) or a network address (`10.0.0.42`,
`10.0.0.42:3333`, or `drive-harness.local`) and get back an object with the
same read/write API either way.

The difference that matters to callers: a serial link can hardware-reset the
board by pulsing DTR/RTS, and a TCP link cannot. Check `.can_reset` before
waiting out a boot sequence, or you will sit there for 5.5 seconds waiting for
a reboot that never happened.
"""

import socket
import time

DEFAULT_TCP_PORT = 3333


class TcpLink:
    """Serial-like wrapper around a TCP socket."""

    can_reset = False

    def __init__(self, host, port=DEFAULT_TCP_PORT, timeout=0.1):
        self.host, self.port = host, port
        self.sock = socket.create_connection((host, port), timeout=5.0)
        self.sock.settimeout(timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def read(self, n=1):
        try:
            return self.sock.recv(n)
        except socket.timeout:
            return b""
        except OSError:
            return b""

    def write(self, data):
        try:
            return self.sock.sendall(data)
        except OSError:
            return 0

    # No-ops so callers do not need to special-case the transport.
    def setDTR(self, _):
        pass

    def setRTS(self, _):
        pass

    def reset_input_buffer(self):
        t0 = time.time()
        while time.time() - t0 < 0.2:
            if not self.read(4096):
                break

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def is_network_target(target):
    return not target.startswith("/dev/") and not target.startswith("COM")


def open_link(target, baud=115200, timeout=0.1):
    """Open `target` as either a serial port or a TCP connection."""
    if is_network_target(target):
        host, _, port = target.partition(":")
        return TcpLink(host, int(port) if port else DEFAULT_TCP_PORT, timeout)
    import serial
    s = serial.Serial(target, baud, timeout=timeout)
    s.can_reset = True
    return s


def connect_and_sync(target, boot_wait=5.5, quiet=0.4):
    """
    Open the link and get to a known state.

    Over serial that means resetting the board and waiting out the boot scan.
    Over TCP the board is already running, so it just drains anything buffered.
    Returns (link, was_reset).
    """
    link = open_link(target)
    if getattr(link, "can_reset", False):
        link.setDTR(False)
        link.setRTS(True)
        time.sleep(0.15)
        link.setRTS(False)
        time.sleep(0.05)
        link.reset_input_buffer()
        time.sleep(boot_wait)
        link.reset_input_buffer()
        return link, True

    time.sleep(quiet)
    link.reset_input_buffer()
    return link, False
