#include <AGVFlex.h>

// Instantiating the new renamed class
AGVFlex robot;

void setup() {
  robot.begin();
}

void loop() {
  robot.update();
}
