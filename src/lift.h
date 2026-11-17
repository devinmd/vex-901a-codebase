#ifndef LIFT_H
#define LIFT_H

#include <numbers>

// lift geometry
const int LIFT_STAGES = 3;
const double WINCH_RADIUS_IN = 0.4;                                                        // inches
const double LIFT_IN_PER_DEG = 2 * std::numbers::pi * WINCH_RADIUS_IN / 360 * LIFT_STAGES; // inches lifted per degree the lift motor rotates

// lift heights, in inches above the lift's bottom (0)
const double LIFT_BOTTOM_HEIGHT_IN = 6.5;                                 // height of the center of the claw from the floor at the lift's lowest position
const double GRAB_CUP_CONE_LIFT_HEIGHT_IN = 6.55 - LIFT_BOTTOM_HEIGHT_IN; // height of the center of the claw to pick up a cup and cone standing on the floor
const double LIFT_INTAKE_HEIGHT_IN = 2.6;                                 // lift position for grabbing from intake

// automatic lift moves
const int LIFT_VELOCITY_RPM = 150;        // default speed for setHeight (green cartridge max is 200)
const int LIFT_MAX_VELOCITY_RPM = 200;    // full speed for the green cartridge
const double LIFT_SETTLED_RANGE_IN = 0.25; // within this of the target counts as there
const int LIFT_TIMEOUT_MS = 2000;         // default time to wait for the lift to get there

namespace lift {

// zero the lift and make it hold its position, call once with the lift at the bottom
void initialize();

// moves the lift to a height in inches above the bottom
// async = true returns right away so the robot can do other things while the lift moves,
// async = false waits until the lift gets there (or timeout_ms runs out)
void setHeight(double height_in, int timeout_ms = LIFT_TIMEOUT_MS, bool async = true,
               int velocity_rpm = LIFT_VELOCITY_RPM);

// current height in inches above the bottom
double getHeight();

// true once the lift is within LIFT_SETTLED_RANGE_IN of the last setHeight target
bool isDone();

// waits until the lift reaches the last setHeight target (or timeout_ms runs out)
void waitUntilDone(int timeout_ms = LIFT_TIMEOUT_MS);

} // namespace lift

#endif // LIFT_H
