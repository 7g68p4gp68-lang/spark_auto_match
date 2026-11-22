#!/usr/bin/env python3
"""Test one small upward SwiftPro movement while recording controller replies."""

import time

import serial


def collect(port, seconds):
    deadline = time.monotonic() + seconds
    chunks = []
    while time.monotonic() < deadline:
        chunk = port.read(256)
        if chunk:
            chunks.append(chunk)
    return b"".join(chunks).decode("ascii", errors="replace")


def main():
    with serial.Serial("/dev/ttyACM0", 115200, timeout=0.2) as port:
        time.sleep(2.0)
        port.reset_input_buffer()
        port.write(b"M2120 V0.2\r\n")
        print("before:\n", collect(port, 1.0))
        port.write(b"M17\r\n")
        print("attach:\n", collect(port, 0.5))
        port.write(b"G0 X140.00 Y35.00 Z100.00 F2000\r\n")
        print("after command:\n", collect(port, 3.0))
        port.write(b"M2120 V0\r\n")


if __name__ == "__main__":
    main()
