// Host-side tests for firmware/esp32_main/src/main.cpp (no ESP32 needed).
//   g++ -std=gnu++17 -Istub test_firmware.cpp -o test_firmware && ./test_firmware
#include "../../firmware/esp32_main/src/main.cpp"

namespace sim {
std::vector<std::string> events;
std::deque<std::string> rx;
unsigned long ms = 0;
int adc[40];
}
FakeSerial Serial;

static int failures = 0, checks = 0;
#define CHECK(cond, msg) do { checks++; if (!(cond)) { failures++; printf("  FAIL: %s\n", msg); } } while (0)

// pin -> joint index
static int jointOfPin(int pin) { for (int i = 0; i < NUM_SERVOS; i++) if (SERVO_PINS[i] == pin) return i; return -1; }

struct Write { int joint, angle; };
static std::vector<Write> writes() {
  std::vector<Write> w;
  for (auto &e : sim::events) if (e[0] == 'W') { int p, a; sscanf(e.c_str(), "W %d %d", &p, &a); w.push_back({jointOfPin(p), a}); }
  return w;
}
static std::vector<std::string> output() {
  std::vector<std::string> o;
  for (auto &e : sim::events) if (e[0] == 'O') o.push_back(e.substr(2));
  return o;
}
static void send(const std::string &line) { sim::events.clear(); sim::rx.push_back(line); loop(); }
static int adcFor(int fb, int angle) { return adcAt0[fb] + (adcAt180[fb] - adcAt0[fb]) * angle / 180; }
static void setServoAdc(const int servoAngles[5]) { for (int i = 0; i < 5; i++) sim::adc[FEEDBACK_PINS[i]] = adcFor(i, servoAngles[i]); }
static bool has(const std::vector<std::string> &v, const std::string &prefix) {
  for (auto &l : v)
    if (l.rfind(prefix, 0) == 0) return true;
  return false;
}

int main() {
  for (int &a : sim::adc) a = 2048;
  setup();

  printf("[1] startup puts every servo at HOME\n");
  {
    auto w = writes();
    CHECK(w.size() == 6, "6 servo writes at startup");
    for (auto &x : w) CHECK(x.angle == HOME_POSE[x.joint], "startup angle == HOME");
    CHECK(has(output(), "# ROBOT READY"), "ready message");
  }

  printf("[2] small move goes straight to target, wrist -> base, with S3 inverted and S6 mirrored\n");
  {
    send("100,95,85,90,92");
    auto w = writes();
    std::vector<int> order; for (auto &x : w) order.push_back(x.joint);
    CHECK((order == std::vector<int>{4, 2, 1, 5, 0}), "order S5,S3,S2,S6,S1 (S4 unchanged, skipped)");
    for (auto &x : w) {
      if (x.joint == 2) CHECK(x.angle == 180 - 85, "S3 = 180 - s3");
      if (x.joint == 5) CHECK(x.angle == 180 - 95, "S6 = 180 - S2");
    }
    CHECK(has(output(), "# OK 100,95,85,90,92"), "OK echo");
  }

  printf("[3] MATLAB's extra blank line is ignored\n");
  {
    send("");
    CHECK(writes().empty(), "no servo writes");
    CHECK(!has(output(), "# ERR"), "no error");
  }

  printf("[4] big move goes through HOME first (A1 -> HOME -> B1)\n");
  {
    send("30,120,150,60,170");
    auto w = writes();
    // first block of writes must be HOME values
    bool homeFirst = !w.empty() && w[0].angle == HOME_POSE[w[0].joint];
    CHECK(homeFirst, "first write is a HOME angle");
    int target[6] = {30, 120, 30, 60, 170, 60};
    for (int j = 0; j < 6; j++) {
      int last = -1; for (auto &x : w) if (x.joint == j) last = x.angle;
      CHECK(last == target[j], "every joint ends at its target");
    }
  }

  printf("[5] out-of-range angles are clamped\n");
  {
    send("-20,250,90,90,90");
    int s1 = -1, s2 = -1; for (auto &x : writes()) { if (x.joint == 0) s1 = x.angle; if (x.joint == 1) s2 = x.angle; }
    CHECK(s1 == 0 && s2 == 180, "S1 clamped to 0, S2 to 180");
  }

  printf("[6] bad packets are rejected without moving\n");
  {
    for (auto bad : {"garbage", "1,2,3", "STOPP"}) {
      send(bad);
      CHECK(writes().empty(), "no move on bad packet");
      CHECK(has(output(), "# ERR"), "error reported");
    }
  }

  printf("[7] feedback: calibrated ADC -> angles in MATLAB frame, S3 inversion undone\n");
  {
    send("HOME");
    int servo[5] = {45, 120, 180 - 70, 10, 170};  // physical servo angles; S3 servo = 180 - 70
    setServoAdc(servo);
    sim::events.clear(); sim::ms += FEEDBACK_PERIOD_MS + 1; loop();
    auto o = output();
    CHECK(!o.empty(), "periodic feedback line sent");
    int f[6] = {0};
    if (!o.empty()) sscanf(o.back().c_str(), "%d,%d,%d,%d,%d,%d", &f[0], &f[1], &f[2], &f[3], &f[4], &f[5]);
    int expect[5] = {45, 120, 70, 10, 170};
    for (int i = 0; i < 5; i++) CHECK(abs(f[i] - expect[i]) <= 1, "feedback angle within 1 deg");
    CHECK(f[5] == HOME_POSE[5], "f6 = commanded S6");
  }

  printf("[8] CAL prints raw ADC values\n");
  {
    send("cal");
    CHECK(has(output(), "# RAW FB1="), "raw line");
  }

  printf("[9] every packet from MATLAB's chess table is accepted (packets.txt, if present)\n");
  {
    FILE *fp = fopen("packets.txt", "r");
    if (!fp) printf("  (skipped: run tests/matlab/export_packets.m first)\n");
    else {
      char buf[128]; int n = 0, ok = 0;
      while (fgets(buf, sizeof buf, fp)) {
        std::string line(buf); while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();
        if (line.empty()) continue;
        send(line); n++;
        if (has(output(), "# OK ") && !has(output(), "# ERR")) ok++;
      }
      fclose(fp);
      printf("  %d/%d packets accepted\n", ok, n);
      CHECK(n > 0 && ok == n, "all MATLAB packets accepted");
    }
  }

  printf("\n%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
