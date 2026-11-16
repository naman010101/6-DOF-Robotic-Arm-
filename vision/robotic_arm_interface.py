"""Python serial interface for the ESP32 arm firmware (firmware/esp32_main).

Sends the same packet MATLAB sends ("s1,s2,s3,s4,s5") and reads the
"f1,...,f6" feedback lines. Needs pyserial: pip install pyserial

Examples:
    python robotic_arm_interface.py --port COM9 --home
    python robotic_arm_interface.py --port COM9 --angles 90 100 120 80 60
    python robotic_arm_interface.py --port COM9 --feedback
"""

import argparse
import time

import serial

DEFAULT_PORT = "COM9"
BAUD = 115200


class ArmInterface:
    def __init__(self, port=DEFAULT_PORT, baud=BAUD, timeout=1.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2.0)  # the ESP32 resets when the port opens
        self.ser.reset_input_buffer()

    def send_angles(self, s1, s2, s3, s4, s5):
        """Send five joint angles (0-180, MATLAB frame)."""
        angles = [max(0, min(180, int(round(a)))) for a in (s1, s2, s3, s4, s5)]
        self.ser.write((",".join(map(str, angles)) + "\n").encode())
        return angles

    def go_home(self):
        self.ser.write(b"HOME\n")

    def read_feedback(self, timeout=2.0):
        """Return the next feedback line as six ints, or None on timeout."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) == 6:
                try:
                    return [int(p) for p in parts]
                except ValueError:
                    continue
        return None

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def main():
    parser = argparse.ArgumentParser(description="Control the 6-DOF arm over serial.")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--angles", type=float, nargs=5, metavar=("S1", "S2", "S3", "S4", "S5"))
    parser.add_argument("--home", action="store_true", help="move to the home pose")
    parser.add_argument("--feedback", action="store_true", help="print one feedback reading")
    args = parser.parse_args()

    with ArmInterface(args.port) as arm:
        if args.home:
            arm.go_home()
        if args.angles:
            print("Sent:", arm.send_angles(*args.angles))
        if args.home or args.angles or args.feedback:
            print("Feedback:", arm.read_feedback(timeout=5.0))


if __name__ == "__main__":
    main()
