// #include <Arduino.h>
// #include <Servo.h>

// Servo baseServo;    // joint 0
// Servo elbowServo;   // joint 1
// Servo wristServo;   // joint 2

// // Link lengths (cm)
// float a1 = 15.8;
// float a2 = 14.2;

// const float MAX_REACH = a1 + a2;
// const float MIN_REACH = sqrt(a1 * a1 + a2 * a2); // elbow ±90° limit

// float x, z, y;

// void setup()
// {
//   Serial.begin(9600);

//   baseServo.attach(11);
//   elbowServo.attach(10);
//   wristServo.attach(9);

//   baseServo.write(90);
//   elbowServo.write(90);
//   wristServo.write(90);

//   Serial.println("✅ System Ready");
//   Serial.println("Enter  X Y Z (example: 15 10 5)");
// }

// void inverseKinematics(float x ,float z, float y)
// {
//   if (z < 0)
//     z = 0;

//   float r = sqrt(z * z + y * y);
//   float R = sqrt(r*r + x*x);

//   int s = (y < 0) ? -1 : 1;

//   // ---------- Law of Cosines ----------
//   float theta1 = atan2(z, y);

//   float theta2 = acos((a1 * a1 + r * r - a2 * a2) / (2 * a1 * r)); // radians
//   float theta3 = acos((r * r - a1 * a1 - a2 * a2) / (2 * a1 * a2)); // radians
  
//   float theta4 = atan2(x, R); // radians

//   float alpha = theta1 + s * theta2;
//   float gamma = PI / 2 - alpha;
//   float beta  = theta3; 

//   // ---------- Convert to degrees ----------
//   float baseDeg  = degrees(theta4);
//   float elbowDeg = degrees(gamma);
//   float wristDeg  = degrees(beta);

//   // ---------- Map to servo angles ----------
//   int base = constrain(baseDeg, 0, 180);
//   int elbow = constrain(90 + elbowDeg, 0, 180);
//   int wrist = constrain( s * wristDeg, 0, 180);

//   baseServo.write(base);
//   elbowServo.write(elbow);
//   wristServo.write(wrist);

//   Serial.print("θ1 = ");
//   Serial.print(baseDeg);
//   Serial.print("°, θ2 = ");
//   Serial.print(elbowDeg);
//   Serial.print("°, θ3 = ");
//   Serial.print(wristDeg);
//   Serial.println("°");
// }

// void loop()
// {
//   if (Serial.available() > 0)
//   {
//     float x_in = Serial.parseFloat();
//     float z_in = Serial.parseFloat();
//     float y_in = Serial.parseFloat();

//     while (Serial.available())
//       Serial.read();

//     Serial.println("\n----------------------------------------");
//     Serial.print("INPUT RECEIVED: X=");
//     Serial.print(x_in);
//     Serial.print("  Y=");
//     Serial.print(y_in);
//     Serial.print("  Z=");
//     Serial.println(z_in);

//     // ---------- FLOOR SAFETY ----------
//     z = z_in;
//     y = max(y_in, 0.0);
//     x = x_in;

//     float r = sqrt(z * z + y * y);
//     float R = sqrt(r*r + x*x);

//     if (y_in < 0)
//     {
//       Serial.println("SAFETY WARNING: Target is underground!");
//     }

//     // if (r >(MAX_REACH  + 0.01))
//     // {
//     //   Serial.println("⚠️ RANGE WARNING: Target too far");
//     //   return;
//     // }

//     // if (r <(MIN_REACH - 0.01))
//     // {
//     //   Serial.println("⚠️ JOINT WARNING: Target too close");
//     //   return;
//     // }

//     inverseKinematics(x, z, y);
//   }
// }

#include <Arduino.h>
#include <Servo.h>

// ---------------- SERVOS ----------------
Servo baseServo;     // joint 0 (yaw)
Servo elbowServo;    // joint 1
Servo wristServo;    // joint 2

// ---------------- LINK LENGTHS (cm) ----------------
float a1 = 15.8;
float a2 = 14.2;

const float MAX_REACH = a1 + a2;
const float MIN_REACH = abs(a1 - a2); // elbow ±90° limit

// ---------------- TARGET ----------------
float x, y, z;

// ---------------- SETUP ----------------
void setup()
{
  Serial.begin(9600);

  baseServo.attach(11);
  elbowServo.attach(10);
  wristServo.attach(9);

  baseServo.write(0);
  elbowServo.write(90);
  wristServo.write(90);

  Serial.println("✅ System Ready");
  Serial.println("Enter X Y Z   (example: 15 5 10)");
}

// ---------------- INVERSE KINEMATICS ----------------
void inverseKinematics(float x, float y, float z)
{
  // ---------- FLOOR SAFETY ----------
  if (z < 0)
    z = 0;

  // ---------- PLANAR DISTANCE ----------
  float R = sqrt(x * x + y * y);
  float d = sqrt(R * R + z * z);

  // ---------- AUTO-CLAMP TARGET ----------
  if (d > MAX_REACH)
  {
    float scale = MAX_REACH / d;
    R *= scale;
    z *= scale;
    d = MAX_REACH;
    Serial.println("⚠️ Target too far → clamped to MAX reach");
  }
  else if (d < MIN_REACH)
  {
    float scale = MIN_REACH / d;
    R *= scale;
    z *= scale;
    d = MIN_REACH;
    Serial.println("⚠️ Target too close → clamped to MIN reach");
  }
  if (z == 0 && R >= MAX_REACH - 0.5)
{
  elbowServo.write(0);
  wristServo.write(180);
  return;
}

  // ---------- ARM IK (R–Z PLANE) ----------
  float thetaBase = atan2(y, x);

  float theta1 = atan2(z, R);

  float theta2 = acos((d * d - a1 * a1 - a2 * a2) / (2 * a1 * a2));

  

  // 🔒 ELBOW SINGULARITY PROTECTION
  // float theta2_min = radians(5);   // 5° minimum bend
  // if (theta2 < theta2_min)
  //   theta2 = theta2_min;
 
  float wrist = theta2;

  float elbow = theta1 - atan2(a2 * sin(theta2), a1 + a2 * cos(theta2));

  // ---------- CONVERT TO DEGREES ----------
  float baseDeg = degrees(thetaBase);
  float elbowDeg = degrees(elbow);
  float wristDeg = degrees(wrist);

  // ---------- SERVO MAPPING ----------
  int basePos = constrain(90+ baseDeg, 0, 180);
  int elbowPos = constrain(90 + elbowDeg, 0, 180);
  int wristPos = constrain( 90 +wristDeg, 0, 180);

  baseServo.write(basePos);
  elbowServo.write(elbowPos);
  wristServo.write(wristPos);

  // ---------- DEBUG ----------
  Serial.print("Base = ");
  Serial.print(basePos);
  Serial.print("°, Elbow = ");
  Serial.print(elbowDeg);
  Serial.print("°, Wrist = ");
  Serial.print(wristDeg);
  Serial.println("°");
}

// ---------------- LOOP ----------------
void loop()
{
  if (Serial.available() > 0)
  {
    float x_in = Serial.parseFloat();
    float y_in = Serial.parseFloat();
    float z_in = Serial.parseFloat();

    while (Serial.available())
      Serial.read();

    Serial.println("\n----------------------------------------");
    Serial.print("INPUT:  X=");
    Serial.print(x_in);
    Serial.print("  Y=");
    Serial.print(y_in);
    Serial.print("  Z=");
    Serial.println(z_in);

    x = x_in;
    y = y_in;
    z = z_in;

    if (z_in < 0)
    {
      Serial.println("⚠️ SAFETY: Target below ground");
    }

    inverseKinematics(x, y, z);
  }
}




