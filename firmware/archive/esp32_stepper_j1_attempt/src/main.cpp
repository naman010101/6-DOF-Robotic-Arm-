#include <ESP32Servo.h>

// =====================================================
// SERVO OBJECTS
// =====================================================
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

// =====================================================
// SERVO PINS
// =====================================================
#define SERVO2_PIN 12
#define SERVO3_PIN 14
#define SERVO4_PIN 25
#define SERVO5_PIN 27
#define SERVO6_PIN 18

// =====================================================
// STEPPER DRIVER PINS
// =====================================================
#define STEP_PIN 13
#define DIR_PIN 21

// =====================================================
// STEPPER SETTINGS
// =====================================================

// NEMA17 = 200 full steps/revolution
#define STEPS_PER_REV 200

// Full step = 1
// 1/2      = 2
// 1/4      = 4
// 1/8      = 8
// 1/16     = 16
#define MICROSTEPS 1

#define TOTAL_STEPS_PER_REV (STEPS_PER_REV * MICROSTEPS)

// =====================================================
// ANGLE SETTINGS
// =====================================================
#define MIN_ANGLE 0
#define MAX_ANGLE 180

// We manually define startup position as 90°
#define HOME_ANGLE 90.0

// =====================================================
// CURRENT STEPPER POSITION
// =====================================================

// IMPORTANT:
// Physically place the stepper at 90° before powering ON.
float currentAngle = HOME_ANGLE;

// Position relative to 90° home
long currentStep = 0;

// =====================================================
// STOP FLAG
// =====================================================
bool stopMotor = false;

// =====================================================
// SPEED
// =====================================================
#define MOVE_DELAY 1500

// =====================================================
// CONVERT ANGLE TO STEPS
// =====================================================
long angleToSteps(float angle)
{
  return round(
      (angle / 360.0) * TOTAL_STEPS_PER_REV);
}

// =====================================================
// CHECK FOR STOP COMMAND
// =====================================================
void checkForStop()
{
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');

    command.trim();
    command.toUpperCase();

    if (command == "STOP")
    {
      stopMotor = true;

      Serial.println();
      Serial.println("!!! STOP COMMAND RECEIVED !!!");
    }
  }
}

// =====================================================
// MOVE STEPPER TO ANGLE
// =====================================================
void moveStepperToAngle(float targetAngle)
{
  // -------------------------------------------------
  // SAFETY LIMIT
  // -------------------------------------------------
  if (targetAngle < MIN_ANGLE ||
      targetAngle > MAX_ANGLE)
  {
    Serial.println(
        "ERROR: Enter angle between 0 and 180.");

    return;
  }

  stopMotor = false;

  // -------------------------------------------------
  // HOME = 90°
  //
  // Example:
  // 28°  = -62° from home
  // 120° = +30° from home
  // -------------------------------------------------

  long targetStep =
      angleToSteps(targetAngle - HOME_ANGLE);

  // -------------------------------------------------
  // CALCULATE MOVEMENT
  // -------------------------------------------------

  long stepsToMove =
      targetStep - currentStep;

  Serial.print("Moving J1 from ");
  Serial.print(currentAngle);
  Serial.print("° to ");
  Serial.print(targetAngle);
  Serial.println("°");

  // Already at target
  if (stepsToMove == 0)
  {
    Serial.println("Already at target.");
    return;
  }

  // -------------------------------------------------
  // DIRECTION
  // -------------------------------------------------

  bool direction;

  if (stepsToMove > 0)
  {
    direction = HIGH;
  }
  else
  {
    direction = LOW;
    stepsToMove = -stepsToMove;
  }

  digitalWrite(DIR_PIN, direction);

  // -------------------------------------------------
  // MOVE
  // -------------------------------------------------

  for (long i = 0; i < stepsToMove; i++)
  {
    // Check for STOP before every step
    checkForStop();

    if (stopMotor)
    {
      Serial.println("!!! MOTOR STOPPED !!!");

      Serial.print(
          "Current estimated angle = ");

      Serial.print(currentAngle);
      Serial.println("°");

      return;
    }

    // STEP pulse
    digitalWrite(STEP_PIN, HIGH);

    delayMicroseconds(MOVE_DELAY);

    digitalWrite(STEP_PIN, LOW);

    delayMicroseconds(MOVE_DELAY);

    // Update position AFTER the step
    if (direction == HIGH)
    {
      currentStep++;
    }
    else
    {
      currentStep--;
    }

    // Update estimated angle
    currentAngle =
        HOME_ANGLE +
        ((float)currentStep /
         TOTAL_STEPS_PER_REV) *
            360.0;
  }

  // -------------------------------------------------
  // TARGET REACHED
  // -------------------------------------------------

  currentAngle = targetAngle;
  currentStep = targetStep;

  Serial.print("J1 reached ");
  Serial.print(currentAngle);
  Serial.println("°");
}

// =====================================================
// MOVE ALL ROBOT JOINTS
// =====================================================
void moveRobot(
    float a1,
    int a2,
    int a3,
    int a4,
    int a5,
    int a6)
{
  // J1 - STEPPER
  moveStepperToAngle(a1);

  delay(100);

  // J5
  servo5.write(a5);

  delay(100);

  // J4
  servo4.write(a4);

  delay(100);

  // J3
  servo3.write(a3);

  delay(100);

  // J2 + J6
  servo2.write(a2);
  servo6.write(a6);

  delay(500);
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);

  // =================================================
  // STEPPER
  // =================================================

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  // =================================================
  // SERVOS
  // =================================================

  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);
  servo6.attach(SERVO6_PIN);

  // =================================================
  // ALL SERVOS → 90°
  // =================================================

  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  servo5.write(90);
  servo6.write(90);

  // =================================================
  // STARTUP
  // =================================================

  Serial.println();
  Serial.println("=================================");
  Serial.println("ROBOT READY");
  Serial.println("=================================");

  Serial.println("J1 Stepper = 90°");
  Serial.println("J2 Servo   = 90°");
  Serial.println("J3 Servo   = 90°");
  Serial.println("J4 Servo   = 90°");
  Serial.println("J5 Servo   = 90°");
  Serial.println("J6 sServo   = 90°");

  Serial.println();
  Serial.println("IMPORTANT:");
  Serial.println("Physically place J1 at 90° before startup.");

  Serial.println();
  Serial.println("Enter J1 angle: 0-180");
  Serial.println("Enter STOP to stop the stepper.");

  Serial.println("=================================");
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  if (Serial.available() > 0)
  {
    String command =
        Serial.readStringUntil('\n');

    command.trim();
    command.toUpperCase();

    // =================================================
    // STOP
    // =================================================

    if (command == "STOP")
    {
      stopMotor = true;

      Serial.println("STOP command received.");

      return;
    }

    // =================================================
    // ANGLE
    // =================================================

    float targetAngle =
        command.toFloat();

    if (targetAngle >= 0 &&
        targetAngle <= 180)
    {
      moveStepperToAngle(targetAngle);
    }
    else
    {
      Serial.println();
      Serial.println(
          "Invalid input.");

      Serial.println(
          "Enter angle 0-180 or STOP.");
    }

    Serial.println();
    Serial.println(
        "Enter next angle or STOP:");
  }
}