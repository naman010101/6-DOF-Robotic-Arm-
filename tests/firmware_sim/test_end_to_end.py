"""End-to-end test: vision/robotic_arm_interface.py <-> virtual serial port <-> simulated ESP32 firmware.

Linux/WSL only (uses a pseudo-terminal):
    g++ -std=gnu++17 -Istub sim_main.cpp -o sim_firmware
    python3 test_end_to_end.py ./sim_firmware
"""
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "vision"))
from robotic_arm_interface import ArmInterface  # noqa: E402

sim = sys.argv[1] if len(sys.argv) > 1 else "./sim_firmware"
master, slave = os.openpty()
proc = subprocess.Popen([sim], stdin=master, stdout=master)
fails = 0


def check(cond, msg):
    global fails
    print(("  ok   " if cond else "  FAIL ") + msg)
    fails += not cond


try:
    with ArmInterface(os.ttyname(slave), timeout=0.5) as arm:
        fb = arm.read_feedback(timeout=2)
        check(fb is not None and len(fb) == 6, f"periodic feedback arrives: {fb}")

        for cmd in ([90, 100, 120, 80, 60], [30, 120, 150, 60, 170], [135, 70, 40, 100, 20]):
            sent = arm.send_angles(*cmd)
            arm.ser.reset_input_buffer()
            fb = None
            for _ in range(10):  # skip feedback lines sent before the move finished
                fb = arm.read_feedback(timeout=2)
                if fb and all(abs(a - b) <= 1 for a, b in zip(fb[:5], sent)):
                    break
            check(fb is not None and all(abs(a - b) <= 1 for a, b in zip(fb[:5], sent)),
                  f"sent {sent} -> feedback {fb}")
            check(fb is not None and fb[5] == 180 - sent[1], "f6 = 180 - S2 (mirrored servo)")

        sent = arm.send_angles(-30, 400, 90, 90, 90)
        check(sent == [0, 180, 90, 90, 90], f"Python clamps before sending: {sent}")

        arm.grip_close()
        line = arm.wait_for("# GRIP", timeout=3)
        check(line == "# GRIP 110", f"CLOSE -> {line}")
        arm.grip(75)
        line = arm.wait_for("# GRIP", timeout=3)
        check(line == "# GRIP 75", f"GRIP 75 -> {line}")

        # full pick-and-place: square a1 from the measured table -> a box pose
        log = []
        orig = arm.wait_for
        arm.wait_for = lambda prefix, timeout=10.0: log.append(orig(prefix, timeout)) or log[-1]
        arm.pick_and_place([31, 88, 25, 110, 150], [150, 70, 60, 120, 90])
        arm.wait_for = orig
        expect = ["# GRIP 40", "# OK 31,88,25,110,150", "# GRIP 110", "# HOME reached",
                  "# OK 150,70,60,120,90", "# GRIP 40", "# HOME reached"]
        check(log == expect, "pick-and-place order: open, pick, close, lift, place, open, home")
        if log != expect:
            print("    got:", log)

        arm.go_home()
        fb = None
        for _ in range(10):
            fb = arm.read_feedback(timeout=2)
            if fb and all(abs(a - 90) <= 1 for a in fb[:5]):
                break
        check(fb is not None and all(abs(a - 90) <= 1 for a in fb[:5]), f"HOME -> feedback {fb}")
finally:
    proc.kill()

print(f"\n{'PASS' if fails == 0 else f'{fails} FAILED'}")
sys.exit(1 if fails else 0)
