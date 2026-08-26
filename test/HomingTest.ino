// Simple homing test sketch for the homing routine.
// This can be compiled and uploaded to the Arduino as a quick smoke test.
// It simulates the sequence by printing the state transitions.

#include <Arduino.h>

struct AxisHomingState {
  bool firstPassDone;
  bool secondPassDone;
  long measuredCounts;
};

struct MockMotor {
  int speed;
  bool stopped;

  void setSpeed(int value) {
    speed = value;
    stopped = false;
  }

  void stop() {
    speed = 0;
    stopped = true;
  }
};

struct MockSwitch {
  bool pressed;
  bool justPressedEdge;

  void update(bool value) {
    pressed = value;
    justPressedEdge = value && !pressed;
  }

  bool justPressed() {
    bool edge = justPressedEdge;
    justPressedEdge = false;
    return edge;
  }
};

struct MockEncoder {
  long count;

  void reset() { count = 0; }

  long position() const { return count; }

  void setPosition(long value) { count = value; }
};

bool homeAxis(MockMotor& motor, MockSwitch& firstSwitch,
              MockSwitch& secondSwitch, MockEncoder& encoder,
              int firstPassSpeed, int secondPassSpeed, int firstDirection,
              int secondDirection, AxisHomingState& axisState) {
  if (!axisState.firstPassDone) {
    motor.setSpeed(firstDirection * firstPassSpeed);
    if (firstSwitch.justPressed()) {
      motor.stop();
      encoder.reset();
      axisState.firstPassDone = true;
      Serial.println("First pass hit -> reset encoder");
    }
    return false;
  }

  if (!axisState.secondPassDone) {
    motor.setSpeed(secondDirection * secondPassSpeed);
    if (secondSwitch.justPressed()) {
      motor.stop();
      axisState.measuredCounts = encoder.position();
      axisState.secondPassDone = true;
      Serial.println("Second pass hit -> measure span");
      return true;
    }
  }

  return false;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Homing test sketch started");
}

void loop() {
  static MockMotor mot;
  static MockSwitch swFirst;
  static MockSwitch swSecond;
  static MockEncoder enc;
  static AxisHomingState axis{false, false, 0};
  static bool done = false;

  if (done) {
    return;
  }

  Serial.println("Running homing step");

  if (homeAxis(mot, swFirst, swSecond, enc, 10, 5, -1, 1, axis)) {
    done = true;
    Serial.print("Measured counts: ");
    Serial.println(axis.measuredCounts);
  }

  // Simulate a switch hit on the first pass after a few cycles.
  static int ticks = 0;
  ticks++;
  if (ticks == 3) {
    swFirst.update(true);
    enc.setPosition(120);
  }

  if (ticks == 6) {
    swSecond.update(true);
    enc.setPosition(240);
  }

  delay(100);
}
