#ifndef HELPERS_H
#define HELPERS_H

#include "main.h" // IWYU pragma: keep

// helpers
bool isMotorJammed(pros::Motor &m);
void wingMacro();
void testMotors();

void lockMacro();

#endif // HELPERS_H