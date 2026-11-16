#include <ESP32Servo.h>

         Servo myServo;

void setup()
{

  // Allow PWM signal range
  ESP32PWM::allocateTimer(0);

  // Servo signal wire connected to GPIO 18
  myServo.setPeriodHertz(50);

  myServo.attach(18, 500, 2400);
}

void loop()
{

  myServo.write(0);
  delay(1000);

  myServo.write(90);
  delay(1000);

  myServo.write(180);
  delay(1000);
}