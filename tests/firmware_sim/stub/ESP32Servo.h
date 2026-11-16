#pragma once
#include "Arduino.h"
struct ESP32PWM { static void allocateTimer(int) {} };
struct Servo {
  int pin = -1;
  void setPeriodHertz(int) {}
  void attach(int p, int, int) { pin = p; }
  void write(int a) { sim::events.push_back("W " + std::to_string(pin) + " " + std::to_string(a)); }
};
