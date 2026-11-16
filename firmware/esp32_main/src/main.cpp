/*
 * 6-DOF Robotic Arm - ESP32 main firmware (six servos, five analog feedback channels)
 *
 * Serial protocol, 115200 baud, one command per line:
 *   PC -> ESP32
 *     s1,s2,s3,s4,s5   Five joint angles (0-180) in MATLAB's frame, exactly what
 *                      matlab/hardware/arm_control_app.m sends.
 *     HOME             Move to HOME_POSE.
 *     CAL              Print the raw ADC value of every feedback channel (for calibration).
 *   ESP32 -> PC
 *     f1,f2,f3,f4,f5,f6  Measured joint angles in the same frame as s1..s5, sent every
 *                        FEEDBACK_PERIOD_MS and after every move. Joint 6 has no feedback
 *                        wire, so its commanded angle is reported.
 *                        matlab/hardware/moving_physical_arm.m reads these lines.
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

const int JOINT_DELAY_MS = 100;  // pause between joints (wrist -> base order)
const int SETTLE_MS = 500;       // pause after a move before reading feedback
const unsigned long FEEDBACK_PERIOD_MS = 200;

// ===================== State =====================
Servo servos[NUM_SERVOS];
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

// Moves wrist -> base: S5, S4, S3, S2 together with S6, then S1.
void moveRobot(const int target[NUM_SERVOS])
{
  bool moved = false;

  const int wristToElbow[] = {4, 3, 2};
  for (int k = 0; k < 3; k++)
  {
    if (writeJoint(wristToElbow[k], target[wristToElbow[k]]))
    {
      moved = true;
      delay(JOINT_DELAY_MS);
    }
  }

  bool s2 = writeJoint(1, target[1]);
  bool s6 = writeJoint(5, target[5]);  // was missing before: S6 never received its angle
  if (s2 || s6)
  {
    moved = true;
    delay(JOINT_DELAY_MS);
  }

  if (writeJoint(0, target[0]))
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

  int s[5];
  if (sscanf(msg.c_str(), "%d,%d,%d,%d,%d", &s[0], &s[1], &s[2], &s[3], &s[4]) != 5)
  {
    Serial.print("# ERR expected s1,s2,s3,s4,s5 but got: ");
    Serial.println(msg);
    return;
  }

  int target[NUM_SERVOS];
  commandToServo(s, target);

  if (largestJointChange(target) > HOME_FIRST_THRESHOLD_DEG)
    moveRobot(HOME_POSE);

  moveRobot(target);

  Serial.printf("# OK %d,%d,%d,%d,%d\n", s[0], s[1], s[2], s[3], s[4]);
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

  delay(SETTLE_MS);
  Serial.println("# ROBOT READY - send s1,s2,s3,s4,s5 | HOME | CAL");
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
