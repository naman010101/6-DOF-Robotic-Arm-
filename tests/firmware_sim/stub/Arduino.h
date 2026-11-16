// Minimal host-side stand-ins for the Arduino API, enough to run firmware/esp32_main on a PC.
#pragma once
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <string>
#include <vector>
using std::max;
using std::min;
#define constrain(a, l, h) ((a) < (l) ? (l) : ((a) > (h) ? (h) : (a)))

class String {
public:
  std::string s;
  String() {}
  String(const char *c) : s(c) {}
  String(const std::string &x) : s(x) {}
  void trim() {
    size_t b = s.find_first_not_of(" \r\n\t"), e = s.find_last_not_of(" \r\n\t");
    s = (b == std::string::npos) ? "" : s.substr(b, e - b + 1);
  }
  void toUpperCase() { for (auto &c : s) c = (char)toupper((unsigned char)c); }
  unsigned length() const { return (unsigned)s.size(); }
  const char *c_str() const { return s.c_str(); }
  bool operator==(const char *o) const { return s == o; }
};

namespace sim {
extern std::vector<std::string> events;  // "W <pin> <angle>" for servo writes, "O <text>" for serial output
extern std::deque<std::string> rx;       // lines waiting to be read by the firmware
extern unsigned long ms;
extern int adc[40];
}

struct FakeSerial {
  std::string line;
  void begin(int) {}
  int available() { return !sim::rx.empty(); }
  String readStringUntil(char) { auto x = sim::rx.front(); sim::rx.pop_front(); return String(x); }
  void out(const std::string &t) {
    line += t;
    size_t n;
    while ((n = line.find('\n')) != std::string::npos) { sim::events.push_back("O " + line.substr(0, n)); line.erase(0, n + 1); }
  }
  void print(const char *c) { out(c); }
  void print(const String &c) { out(c.s); }
  void println(const char *c = "") { out(std::string(c) + "\n"); }
  void println(const String &c) { out(c.s + "\n"); }
  void printf(const char *f, ...) {
    char b[256]; va_list a; va_start(a, f); vsnprintf(b, sizeof b, f, a); va_end(a); out(b);
  }
};
extern FakeSerial Serial;

inline void delay(int d) { sim::ms += d; }
inline unsigned long millis() { return sim::ms; }
inline int analogRead(int p) { return sim::adc[p]; }
inline void analogReadResolution(int) {}
