#!/usr/bin/env python3
"""
car_console.py - talk to the car from the laptop (USB cable or Bluetooth COM port)

  * shows everything the car prints, and sends what you type
  * telemetry (after "T 10") is saved to logs/run_<time>.csv automatically
  * a DUMP from the car is saved to saved_paths/path_data_<time>.h automatically
  * !upload <file>   sends a path_data.h (or x,y,flags CSV) to the car's RAM
  * !quit            exit

Install once:   pip install pyserial matplotlib
Run:            python car_console.py              (lists ports)
                python car_console.py COM5         (Windows USB, 115200)
                python car_console.py COM7 9600    (Bluetooth COM port)
                python car_console.py /dev/ttyACM0 (Linux/macOS)
"""
import os, re, sys, threading, time
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial missing:  pip install pyserial")

HERE = os.path.dirname(os.path.abspath(__file__))


def stamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


class Console:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.running = True
        self.csv = None
        self.header = None
        self.dump = None  # list of lines while a DUMP is being received

    # ---------------------------------------------------------------- reading
    def reader(self):
        buf = b""
        while self.running:
            try:
                data = self.ser.read(256)
            except serial.SerialException as e:
                print(f"\n[serial error: {e}]")
                self.running = False
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self.handle_line(raw.decode(errors="replace").rstrip("\r"))

    def handle_line(self, line):
        # path dump capture
        if line.startswith("// ==== PATH BEGIN"):
            self.dump = []
            print("[receiving path dump ...]")
            return
        if self.dump is not None:
            if line.startswith("// ==== PATH END"):
                os.makedirs(os.path.join(HERE, "saved_paths"), exist_ok=True)
                fn = os.path.join(HERE, "saved_paths", f"path_data_{stamp()}.h")
                with open(fn, "w") as f:
                    f.write("\n".join(self.dump) + "\n")
                n = sum(1 for l in self.dump if l.strip().startswith("{"))
                print(f"[path saved: {fn}  ({n} points)]")
                print("[copy it over cubeide/Team7_Car/Core/Inc/path_data.h and rebuild/flash to make it permanent]")
                self.dump = None
            else:
                self.dump.append(line)
            return
        # telemetry
        if line.startswith("t_ms,"):
            self.header = line
            os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
            if self.csv:
                self.csv.close()
            fn = os.path.join(HERE, "logs", f"run_{stamp()}.csv")
            self.csv = open(fn, "w")
            self.csv.write(line + "\n")
            print(f"[logging telemetry to {fn}]")
            return
        if self.csv and re.match(r"^\d+,", line) and line.count(",") == self.header.count(","):
            self.csv.write(line + "\n")
            self.csv.flush()
            if int(line.split(",")[0]) % 1000 < 110:   # print about once per second
                print(line)
            return
        print(line)

    # ---------------------------------------------------------------- writing
    def send(self, text):
        self.ser.write((text + "\n").encode())

    def upload(self, filename):
        pts = []
        with open(filename) as f:
            for l in f:
                m = re.search(r"\{\s*(-?[\d.]+)f?\s*,\s*(-?[\d.]+)f?\s*,\s*(\d+)\s*\}", l)
                if not m:
                    m = re.match(r"^\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(\d+)\s*$", l)
                if m:
                    pts.append((float(m.group(1)), float(m.group(2)), int(m.group(3))))
        if not pts:
            print("[no points found in file]")
            return
        print(f"[uploading {len(pts)} points ...]")
        self.send("PC")
        time.sleep(0.2)
        slow = self.ser.baudrate <= 19200
        for x, y, fl in pts:
            self.send(f"PA {x:.3f} {y:.3f} {fl}")
            time.sleep(0.03 if slow else 0.005)
        time.sleep(0.3)
        self.send("PATH")

    def run(self):
        threading.Thread(target=self.reader, daemon=True).start()
        print("Connected. Type commands (? for help on the car), !upload <file>, !quit")
        try:
            while self.running:
                cmd = input()
                if cmd.strip() == "!quit":
                    break
                if cmd.startswith("!upload"):
                    parts = cmd.split(maxsplit=1)
                    if len(parts) == 2:
                        self.upload(parts[1].strip())
                    continue
                self.send(cmd)
        except (KeyboardInterrupt, EOFError):
            pass
        self.running = False
        if self.csv:
            self.csv.close()
        self.ser.close()


def main():
    if len(sys.argv) < 2:
        print("Available ports:")
        for p in list_ports.comports():
            print(f"  {p.device:15s} {p.description}")
        print("\nUsage: python car_console.py <port> [baud]   (USB = 115200, Bluetooth = 9600)")
        return
    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    Console(port, baud).run()


if __name__ == "__main__":
    main()
