// Runs firmware/esp32_main as a PC program: serial lines on stdin, serial output on stdout.
// Used by test_end_to_end.py, which connects it to a virtual serial port.
#include "../../firmware/esp32_main/src/main.cpp"
#include <poll.h>
#include <unistd.h>
#include <cmath>

namespace sim {
std::vector<std::string> events;
std::deque<std::string> rx;
unsigned long ms = 0;
int adc[40];
}
FakeSerial Serial;

static void flushOut() {
  for (auto &e : sim::events)
    if (e[0] == 'O') { std::string l = e.substr(2) + "\n"; if (write(1, l.data(), l.size()) < 0) exit(0); }
  sim::events.clear();
}

int main() {
  // pretend every servo sits where it was told: ADC follows the commanded angle
  for (int &a : sim::adc) a = 2048;
  setup();
  flushOut();
  std::string buf;
  while (true) {
    struct pollfd p = {0, POLLIN, 0};
    if (poll(&p, 1, 20) > 0) {
      char c[256];
      ssize_t n = read(0, c, sizeof c);
      if (n <= 0) return 0;
      buf.append(c, n);
      size_t k;
      while ((k = buf.find_first_of("\r\n")) != std::string::npos) { sim::rx.push_back(buf.substr(0, k)); buf.erase(0, k + 1); }
    }
    sim::ms += 20;
    for (int i = 0; i < NUM_FEEDBACK; i++) {
      int servo = current[i];
      sim::adc[FEEDBACK_PINS[i]] = adcAt0[i] + (adcAt180[i] - adcAt0[i]) * servo / 180;
    }
    loop();
    flushOut();
  }
}
