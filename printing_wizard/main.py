#!/usr/bin/env python3

import serial
import serial.tools.list_ports
import time
import sys
import os

from gcode_conversion import make_text_gcode, DEFAULT_FONT, DEFAULT_FONT_SIZE


# ============================
# SERIAL CONFIGURATION
# ============================
BAUDRATE = 115200
SERIAL_TIMEOUT = 0.1  # 100ms timeout for readline

JOG_FEED = 1000      # mm/min
JOG_STEP = 1.0       # mm


if os.name == "nt":
    import msvcrt

    def get_key():
        if msvcrt.kbhit():
            return msvcrt.getch().decode(errors="ignore")
        return None
else:
    import termios
    import tty
    import select

    class UnixKeyReader:
        def __enter__(self):
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            return self

        def __exit__(self, *_):
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

        def get_key(self):
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if r:
                return sys.stdin.read(1)
            return None


# ============================
# SERIAL UTILITIES
# ============================
def list_serial_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
        sys.exit(1)
    return ports


def select_serial_port(ports):
    print("\nAvailable serial ports:")
    for i, p in enumerate(ports):
        print(f"  [Option {i+1}] {p.device} - {p.description}")

    while True:
        try:
            idx = int(input("Select option: "))
            return ports[idx-1].device
        except (ValueError, IndexError):
            print("Invalid selection.")


def open_grbl(port):
    ser = serial.Serial(port, BAUDRATE, timeout=SERIAL_TIMEOUT)
    time.sleep(2)
    ser.write(b"\r\n\r\n")
    time.sleep(1)
    ser.reset_input_buffer()
    print("Connected to GRBL.")
    return ser


# ============================
# GRBL COMMUNICATION
# ============================
def send_line(ser, line):
    ser.write((line + "\n").encode())


def jog(ser, x: float = 0, y: float = 0, z: float = 0):
    cmd = f"$J=G91 X{x:.3f} Y{y:.3f} Z{z:.3f} F{JOG_FEED}"
    send_line(ser, cmd)


def reset_zero(ser, jog_to_start: bool = False) -> None:
    if jog_to_start:
        send_line(ser, "G0 Y0 Z0")
    send_line(ser, "G92 X0 Y0 Z0")


# ============================
# REAL-TIME JOGGING
# ============================
def jogging_loop(ser):
    print("""
Jog the pen to the lower left corner of the label, with the pen a couple of millimeters above it.

Jogging Controls:
  W/S: Raise/lower pen
  A/D: Rotate tape
  R/F: Move pen away or towards tape
  
When ready, press:
  Z: Set work zero and proceed with printing
  Q: Quit program
""")

    step = JOG_STEP

    if os.name == "nt":
        key_reader = None
    else:
        key_reader = UnixKeyReader()
        key_reader.__enter__()

    try:
        while True:
            key = get_key() if os.name == "nt" else key_reader.get_key()

            if not key:
                time.sleep(0.01)
                continue

            key = key.lower()

            if key == "w":
                jog(ser, y=step)
            elif key == "s":
                jog(ser, y=-step)
            elif key == "a":
                jog(ser, x=-step)
            elif key == "d":
                jog(ser, x=step)
            elif key == "r":
                jog(ser, z=step/2)
            elif key == "f":
                jog(ser, z=-step/2)
            elif key == "z":
                reset_zero(ser)
                print("Work zero set.")
                return
            elif key == "q":
                print("Exiting program.")
                sys.exit(0)

    finally:
        if key_reader:
            key_reader.__exit__(None, None, None)


# ============================
# STUPID SIMPLE PING-PONG STREAMER
# ============================
def stream_gcode(ser, gcode_lines):

    # Clear out any leftover 'ok' responses from jogging or zeroing commands
    ser.reset_input_buffer()

    # Normalize input to a list of individual lines
    if isinstance(gcode_lines, str):
        raw_lines = gcode_lines.splitlines()
    else:
        raw_lines = []
        for item in gcode_lines:
            raw_lines.extend(str(item).splitlines())

    for raw_line in raw_lines:
        line = raw_line.strip()

        # Skip empty lines and comments
        if not line or line.startswith(";") or line.startswith("("):
            continue
        if ";" in line:
            line = line.split(";")[0].strip()

        # Send the line to GRBL
        #print(f">> {line}")
        send_line(ser, line)

        # Synchronous block: Wait right here until GRBL answers this exact line
        while True:
            resp = ser.readline().decode(errors="ignore").strip()

            if not resp:
                continue  # Serial timeout hit with no data, keep waiting

            if "ok" in resp:
                break  # GRBL approved it! Break this inner loop and send next line
            elif "error" in resp:
                print(f"[!] GRBL Error: {resp} on line: {line}")
                break  # Log the error and move on to keep it running

    print("G-code completed successfully.")


def print_loop(serial_connection):
    font = input(f'Select font (leave empty for {DEFAULT_FONT}): ')
    font_size = input(f'Select font size in mm (leave empty for {DEFAULT_FONT_SIZE}mm): ')
    should_preview = input('Show preview (y/N)? ').lower() == 'y'
    print()

    while True:
        text = input("Text to print: ").strip()
        gcode = make_text_gcode(text, font, font_size, should_preview)

        # Ensure GRBL clears its buffer before we zero and start
        time.sleep(0.1)
        reset_zero(serial_connection)
        time.sleep(0.1)

        stream_gcode(serial_connection, gcode)

        should_continue = input('\nPrint another (Y), use different settings (D) or quit (anything else)? ').lower()

        if should_continue == 'y':
            print()
            continue
        elif should_continue == 'd':
            print()
            print_loop(serial_connection)
        else:
            print("Exiting")
            sys.exit(0)


def main():
    ports = list_serial_ports()
    port = select_serial_port(ports)
    print("Selected port " + port)
    ser = open_grbl(port)

    jogging_loop(ser)
    print_loop(ser)

    ser.close()
    print("Connection closed.")


if __name__ == "__main__":
    main()
>>>>>>> Implement a simple printing command line utility
