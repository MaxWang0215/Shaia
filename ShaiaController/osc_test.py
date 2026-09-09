"""Smoke test for ShaiaController's OSC API.

Encodes OSC packets with the standard library so it runs without python-osc.
Start the controller (ideally a SIMULATE build) and then:

    python3 osc_test.py            # arms + head
    python3 osc_test.py head       # gestures only
    python3 osc_test.py arms       # notes only
"""

import socket
import struct
import sys
import time

HOST, PORT = "127.0.0.1", 9000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def _pad(b: bytes) -> bytes:
    r = len(b) % 4
    return b if r == 0 else b + b"\0" * (4 - r)


def _encode(address: str, *args) -> bytes:
    tags = ","
    body = b""
    for a in args:
        if isinstance(a, bool):
            raise TypeError("OSC has no bool type here; use int")
        if isinstance(a, int):
            tags += "i"
            body += struct.pack(">i", a)
        elif isinstance(a, float):
            tags += "f"
            body += struct.pack(">f", a)
        else:
            tags += "s"
            body += _pad(a.encode() + b"\0")
    return _pad(address.encode() + b"\0") + _pad(tags.encode() + b"\0") + body


def send(address: str, *args, delay: float = 0.2) -> None:
    print(address, args)
    sock.sendto(_encode(address, *args), (HOST, PORT))
    time.sleep(delay)


def head():
    send("/head", "nod_sway", 120.0, delay=3.0)
    send("/head", "ballet_sway", 56.0, delay=3.0)
    send("/head_stop", delay=0.5)
    send("/head_look", delay=1.0)
    send("/head-commands", "HEADTILT", 0.8, 0.03, delay=0.5)
    send("/head-commands", "MOUTH", 1.0, 0.25, delay=0.3)
    send("/head-commands", "MOUTH", 0.0, 0.25, delay=0.5)
    send("/head-commands", "BEAT", 500, delay=1.0)


def arms():
    send("/arm_home", delay=1.0)
    # F3 up to B5, plus two notes outside the range to exercise transposition.
    for note in (53, 60, 65, 72, 77, 83, 90, 45):
        send("/arm", note, 90, delay=0.7)

    # Rapid repeats of one note take the same-note buffer path: striker only.
    for _ in range(4):
        send("/arm", 67, 110, delay=0.12)

    # Choreo: raise strikers 3 and 4 (bitmask 0b1100) to 45 deg over 300 ms.
    send("/striker", 0x0C, 45, 300, delay=0.5)


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    if which in ("all", "head"):
        head()
    if which in ("all", "arms"):
        arms()
