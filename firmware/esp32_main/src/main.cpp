/*
 * 6-DOF Robotic Arm - ESP32 main firmware (six servos, five analog feedback channels)
 *
 * Serial protocol, 115200 baud, one command per line:
 *   PC -> ESP32
 *     s1,s2,s3,s4,s5   Five joint angles (0-180) in MATLAB's frame, exactly what
 *                      matlab/hardware/arm_control_app.m sends.
 *     s1,s2,s3,s4,s5,g Same, plus a gripper angle applied after the arm has moved.
 *     OPEN / CLOSE     Open or close the gripper (also GRIP OPEN, GRIP CLOSE).
 *     GRIP <0-180>     Set the gripper servo to an angle.
 *     HOME             Move to HOME_POSE.
 *     CAL              Print the raw ADC value of every feedback channel (for calibration).
 *   ESP32 -> PC
 *     f1,f2,f3,f4,f5,f6  Measured joint angles in the same frame as s1..s5, sent every
 *                        FEEDBACK_PERIOD_MS and after every move. Joint 6 has no feedback
 *                        wire, so its commanded angle is reported.
 *                        matlab/hardware/moving_physical_arm.m reads these lines.
 *     # GRIP <angle>     Gripper position after every gripper command.
 *     # ...              Status messages. They start with '#', so parsers can skip them.
 *
 * Note: GPIO 12 is an ESP32 boot-strapping pin. If the board fails to boot with the
 * servo signal wire connected, unplug it while powering up or move servo 2 to another pin.
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// ===================== Pins =====================
const int NUM_SERVOS = 6;
const int NUM_FEEDBACK = 5;

// S1 base, S2 shoulder, S3 elbow, S4 wrist, S5 wrist, S6 mirrors S2
const int SERVO_PINS[NUM_SERVOS] = {13, 12, 14, 25, 27, 18};

// Gripper servo (SG90-type, geared gripper). Not one of the six arm servos: S6 mirrors S2.
// GPIO 26 is a free, boot-safe PWM pin - CHECK that it matches your wiring.
const int GRIPPER_PIN = 26;

// Analog feedback for S1..S5 (all ADC1 pins)
const int FEEDBACK_PINS[NUM_FEEDBACK] = {33, 32, 35, 36, 34};

// ===================== Calibration =====================
// ADC reading with the servo at 0 deg and at 180 deg. The mapping works in either
// direction, so a channel whose reading drops as the angle rises needs no special case.
// PLACEHOLDERS carried over from the previous code: send "CAL" at 0 and 180 deg for
// each servo and replace these with the measured values.
int adcAt0[NUM_FEEDBACK]   = {4095, 3150, 3600, 4095, 3100};
int adcAt180[NUM_FEEDBACK] = {0,    60,   0,    0,    70};

const int ADC_SAMPLES = 8;  // readings averaged per channel to reduce noise

// ===================== Motion =====================
// Servo angles of the home pose. 90 deg everywhere is a placeholder: check it is safe.
const int HOME_POSE[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};

// Go to HOME first when any joint has to move more than this (e.g. square A1 -> B1).
// Small moves, such as the jog buttons in the MATLAB app, go straight to the target.
const int HOME_FIRST_THRESHOLD_DEG = 30;

// Gripper angles - PLACEHOLDERS: find the real open/closed angles with "GRIP <angle>".
const int GRIPPER_OPEN = 40;
const int GRIPPER_CLOSED = 110;
const int GRIPPER_SETTLE_MS = 400;  // time for the fingers to finish moving

const int JOINT_DELAY_MS = 100;  // pause between joints (wrist -> base order)

// Anti-backlash: every joint finishes its move travelling upward. A joint that has to move
// down first overshoots OVERSHOOT_DEG below the target and then comes back up, so the gear
// slack always sits on the same side. That turns backlash into a constant offset, which the
// hand-measured square table already contains. In simulation (matlab/simulation) this raised
// the grasp success rate from 12 % to 99 % with typical hobby-servo backlash.
const bool ANTI_BACKLASH = true;
const int OVERSHOOT_DEG = 4;
const int OVERSHOOT_SETTLE_MS = 120;
const int SETTLE_MS = 500;       // pause after a move before reading feedback
const unsigned long FEEDBACK_PERIOD_MS = 200;

// ===================== State =====================
Servo servos[NUM_SERVOS];
Servo gripper;
int gripperAngle = GRIPPER_OPEN;
int current[NUM_SERVOS];  // last angle written to each servo (replaces prev1..prev6)
unsigned long lastFeedback = 0;

// ---------- MATLAB frame -> servo angles ----------
void commandToServo(const int s[5], int out[NUM_SERVOS])
{
  out[0] = constrain(s[0], 0, 180);
  out[1] = constrain(s[1], 0, 180);
  out[2] = constrain(180 - s[2], 0, 180);  // servo 3 is mounted inverted
  out[3] = constrain(s[3], 0, 180);
  out[4] = constrain(s[4], 0, 180);
  out[5] = 180 - out[1];                   // servo 6 mirrors servo 2
}

// ---------- Movement ----------
bool writeJoint(int i, int angle)
{
  if (angle == current[i])
    return false;
  servos[i].write(angle);
  current[i] = angle;
  return true;
}

// Moves one joint, approaching from below when ANTI_BACKLASH is on.
bool approachJoint(int i, int angle)
{
  if (ANTI_BACKLASH && angle < current[i])
  {
    writeJoint(i, max(0, angle - OVERSHOOT_DEG));
    delay(OVERSHOOT_SETTLE_MS);
  }
  return writeJoint(i, angle);
}

// S2 and its mirror S6 move as a pair; the approach direction follows S2.
bool approachShoulder(int s2, int s6)
{
  if (ANTI_BACKLASH && s2 < current[1])
  {
    int pre = max(0, s2 - OVERSHOOT_DEG);
    writeJoint(1, pre);
    writeJoint(5, 180 - pre);
    delay(OVERSHOOT_SETTLE_MS);
  }
  bool a = writeJoint(1, s2);
  bool b = writeJoint(5, s6);  // was missing before: S6 never received its angle
  return a || b;
}

// Moves wrist -> base: S5, S4, S3, S2 together with S6, then S1.
void moveRobot(const int target[NUM_SERVOS])
{
  bool moved = false;

  const int wristToElbow[] = {4, 3, 2};
  for (int k = 0; k < 3; k++)
  {
    if (approachJoint(wristToElbow[k], target[wristToElbow[k]]))
    {
      moved = true;
      delay(JOINT_DELAY_MS);
    }
  }

  if (approachShoulder(target[1], target[5]))
  {
    moved = true;
    delay(JOINT_DELAY_MS);
  }

  if (approachJoint(0, target[0]))
    moved = true;

  if (moved)
    delay(SETTLE_MS);
}

int largestJointChange(const int target[NUM_SERVOS])
{
  int largest = 0;
  for (int i = 0; i < NUM_SERVOS; i++)
    largest = max(largest, abs(target[i] - current[i]));
  return largest;
}

// ---------- Gripper ----------
void setGripper(int angle)
{
  angle = constrain(angle, 0, 180);
  if (angle != gripperAngle)
  {
    gripper.write(angle);
    gripperAngle = angle;
    delay(GRIPPER_SETTLE_MS);
  }
  Serial.printf("# GRIP %d\n", gripperAngle);
}

// ---------- Feedback ----------
int readAdc(int pin)
{
  long sum = 0;
  for (int k = 0; k < ADC_SAMPLES; k++)
    sum += analogRead(pin);
  return sum / ADC_SAMPLES;
}

int adcToServoAngle(int i, int adc)
{
  long span = adcAt180[i] - adcAt0[i];
  if (span == 0)
    return -1;  // calibration missing
  long angle = lroundf((float)(adc - adcAt0[i]) * 180.0f / span);  // round, don't truncate
  return (int)constrain(angle, 0L, 180L);
}

void sendFeedback()
{
  int a[NUM_FEEDBACK];
  for (int i = 0; i < NUM_FEEDBACK; i++)
    a[i] = adcToServoAngle(i, readAdc(FEEDBACK_PINS[i]));

  int f3 = (a[2] < 0) ? -1 : 180 - a[2];  // undo servo 3 inversion -> MATLAB frame

  Serial.printf("%d,%d,%d,%d,%d,%d\n", a[0], a[1], f3, a[3], a[4], current[5]);
}

void printRawAdc()
{
  Serial.print("# RAW");
  for (int i = 0; i < NUM_FEEDBACK; i++)
    Serial.printf(" FB%d=%d", i + 1, readAdc(FEEDBACK_PINS[i]));
  Serial.println();
}

// ---------- Commands ----------
void handleLine(String msg)
{
  msg.trim();
  if (msg.length() == 0)
    return;  // MATLAB's writeline() adds a second newline; ignore the empty line

  String cmd = msg;
  cmd.toUpperCase();

  if (cmd == "HOME")
  {
    moveRobot(HOME_POSE);
    Serial.println("# HOME reached");
    sendFeedback();
    return;
  }

  if (cmd == "CAL")
  {
    printRawAdc();
    return;
  }

  if (cmd == "OPEN" || cmd == "GRIP OPEN")
  {
    setGripper(GRIPPER_OPEN);
    return;
  }

  if (cmd == "CLOSE" || cmd == "GRIP CLOSE")
  {
    setGripper(GRIPPER_CLOSED);
    return;
  }

  int g = -1;
  if (sscanf(cmd.c_str(), "GRIP %d", &g) == 1)
  {
    setGripper(g);
    return;
  }

  int s[5];
  int n = sscanf(msg.c_str(), "%d,%d,%d,%d,%d,%d", &s[0], &s[1], &s[2], &s[3], &s[4], &g);
  if (n != 5 && n != 6)
  {
    Serial.print("# ERR expected s1,s2,s3,s4,s5[,g] but got: ");
    Serial.println(msg);
    return;
  }

  int target[NUM_SERVOS];
  commandToServo(s, target);

  if (largestJointChange(target) > HOME_FIRST_THRESHOLD_DEG)
    moveRobot(HOME_POSE);

  moveRobot(target);

  Serial.printf("# OK %d,%d,%d,%d,%d\n", s[0], s[1], s[2], s[3], s[4]);
  if (n == 6)
    setGripper(g);  // after the arm has arrived, so it grips/releases in place
  sendFeedback();
}

// ===================== Setup / loop =====================
void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);  // 0..4095

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < NUM_SERVOS; i++)
  {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], 500, 2400);
    servos[i].write(HOME_POSE[i]);
    current[i] = HOME_POSE[i];
  }

  gripper.setPeriodHertz(50);
  gripper.attach(GRIPPER_PIN, 500, 2400);
  gripper.write(GRIPPER_OPEN);

  delay(SETTLE_MS);
  Serial.println("# ROBOT READY - send s1,s2,s3,s4,s5[,g] | HOME | OPEN | CLOSE | GRIP <a> | CAL");
}

void loop()
{
  if (Serial.available() > 0)
    handleLine(Serial.readStringUntil('\n'));

  if (millis() - lastFeedback >= FEEDBACK_PERIOD_MS)
  {
    lastFeedback = millis();
    sendFeedback();
  }
}
