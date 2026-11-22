#!/usr/bin/env python3
"""Read SwiftPro position reports without issuing any movement command."""

import time

import serial


def main():
    with serial.Serial("/dev/ttyACM0", 115200, timeout=0.2) as port:
        time.sleep(2.0)
        port.reset_input_buffer()
        port.write(b"M2120 V0.2\r\n")
        deadline = time.monotonic() + 2.0
        chunks = []
        while time.monotonic() < deadline:
            chunk = port.read(256)
            if chunk:
                chunks.append(chunk)
        port.write(b"M2120 V0\r\n")
        print(b"".join(chunks).decode("ascii", errors="replace"))


if __name__ == "__main__":
    main()
