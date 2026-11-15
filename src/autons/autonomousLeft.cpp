

#include "../motors.h"
#include "autonomous.h"
#include "pros/rtos.hpp"

void autonomousLeft() {

  // start on right side of the toggle
  chassis.setPose(-65, -7, 90);

  // raise lift slightly

  // drive to the goal
  chassis.turnToPoint(-65, -7, 500, {.forwards = true, .maxSpeed = 127}, false);
  chassis.moveToPoint(-65, -7, 1000, {.forwards = true, .maxSpeed = 127}, false);

  // score

  // chassis.turnToPoint(-50, -16, 500, {.forwards = true, .maxSpeed = 127}, false);
  // chassis.moveToPoint(-50, -16, 1000, {.forwards = true, .maxSpeed = 127}, false);

  // chassis.turnToPoint(-62, -24.2, 500, {.forwards = true, .maxSpeed = 127}, false);
  // chassis.moveToPoint(-62, -24.2, 1000, {.forwards = true, .maxSpeed = 127}, false);

  // chassis.turnToPoint(-55.2, -24.4, 500, {.forwards = true, .maxSpeed = 127}, false);
  // chassis.moveToPoint(-55.2, -24.4, 1000, {.forwards = true, .maxSpeed = 127}, false);

  // chassis.turnToPoint(-59.3, -24.3, 500, {.forwards = true, .maxSpeed = 127}, false);
  // chassis.moveToPoint(-59.3, -24.3, 1000, {.forwards = true, .maxSpeed = 127}, false);

  // chassis.turnToPoint(-50.5, -46.5, 500, {.forwards = true, .maxSpeed = 127}, false);
  // chassis.moveToPoint(-50.5, -46.5, 1000, {.forwards = true, .maxSpeed = 127}, false);

  // chassis.turnToPoint(-48.2, -29.5, 500, {.forwards = true, .maxSpeed = 127}, false);
  // chassis.moveToPoint(-48.2, -29.5, 1000, {.forwards = true, .maxSpeed = 127}, false);

  // chassis.turnToPoint(-48.3, -33.6, 500, {.forwards = true, .maxSpeed = 127}, false);
  // chassis.moveToPoint(-48.3, -33.6, 1000, {.forwards = true, .maxSpeed = 127}, false);

  // chassis.turnToPoint(-27.4, -24, 500, {.forwards = true, .maxSpeed = 127}, false);
  // chassis.moveToPoint(-27.4, -24, 1000, {.forwards = true, .maxSpeed = 127}, false);

  // chassis.turnToPoint(-42.2, -24.2, 500, {.forwards = true, .maxSpeed = 127}, false);
  // chassis.moveToPoint(-42.2, -24.2, 1000, {.forwards = true, .maxSpeed = 127}, false);
}
