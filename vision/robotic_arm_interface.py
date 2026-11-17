"""Python serial interface for the ESP32 arm firmware (firmware/esp32_main).

Sends the same packet MATLAB sends ("s1,s2,s3,s4,s5", optionally ",g" for the gripper),
controls the gripper and runs a pick-and-place sequence. Reads the "f1,...,f6" feedback
lines. Needs pyserial: pip install pyserial

Examples:
    python robotic_arm_interface.py --port COM9 --home
    python robotic_arm_interface.py --port COM9 --angles 90 100 120 80 60
    python robotic_arm_interface.py --port COM9 --feedback
    python robotic_arm_interface.py --port COM9 --grip close
    python robotic_arm_interface.py --port COM9 --pick 31 88 25 110 150 --place 90 60 40 120 90
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

    @staticmethod
    def _clamp(a):
        return max(0, min(180, int(round(a))))

    def send_angles(self, s1, s2, s3, s4, s5, gripper=None):
        """Send five joint angles (0-180, MATLAB frame). If gripper is given, the ESP32
        sets the gripper to that angle after the arm has arrived."""
        angles = [self._clamp(a) for a in (s1, s2, s3, s4, s5)]
        packet = angles + ([self._clamp(gripper)] if gripper is not None else [])
        self.ser.write((",".join(map(str, packet)) + "\n").encode())
        return angles

    def go_home(self):
        self.ser.write(b"HOME\n")

    # ---------- gripper ----------
    def grip_open(self):
        self.ser.write(b"OPEN\n")

    def grip_close(self):
        self.ser.write(b"CLOSE\n")

    def grip(self, angle):
        """Set the gripper servo to an angle (use this to find the open/closed values)."""
        angle = self._clamp(angle)
        self.ser.write(f"GRIP {angle}\n".encode())
        return angle

    def wait_for(self, prefix, timeout=10.0):
        """Wait for a status line starting with prefix (e.g. "# OK", "# GRIP", "# HOME")."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line.startswith(prefix):
                return line
        raise TimeoutError(f"no '{prefix}' from the arm within {timeout} s")

    def pick_and_place(self, pick_angles, place_angles, timeout=15.0):
        """Open, move to pick_angles, close, lift via HOME, move to place_angles, open, HOME.
        Angles are the five servo angles of each pose (e.g. from the chess-square table)."""
        steps = [
            (self.grip_open, "# GRIP"),
            (lambda: self.send_angles(*pick_angles), "# OK"),
            (self.grip_close, "# GRIP"),
            (self.go_home, "# HOME"),
            (lambda: self.send_angles(*place_angles), "# OK"),
            (self.grip_open, "# GRIP"),
            (self.go_home, "# HOME"),
        ]
        for action, done in steps:
            action()
            self.wait_for(done, timeout)

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
    parser.add_argument("--grip", help="open, close, or an angle 0-180")
    parser.add_argument("--pick", type=float, nargs=5, metavar="S", help="servo angles of the pick pose")
    parser.add_argument("--place", type=float, nargs=5, metavar="S", help="servo angles of the place pose")
    args = parser.parse_args()
    if bool(args.pick) != bool(args.place):
        parser.error("--pick and --place go together")

    with ArmInterface(args.port) as arm:
        if args.home:
            arm.go_home()
        if args.angles:
            print("Sent:", arm.send_angles(*args.angles))
        if args.grip:
            g = args.grip.lower()
            arm.grip_open() if g == "open" else arm.grip_close() if g == "close" else arm.grip(float(g))
            print(arm.wait_for("# GRIP"))
        if args.pick:
            arm.pick_and_place(args.pick, args.place)
            print("pick-and-place done")
        if args.home or args.angles or args.feedback:
            print("Feedback:", arm.read_feedback(timeout=5.0))


if __name__ == "__main__":
    main()
