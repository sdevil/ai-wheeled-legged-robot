#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "BLDCMotor.h"
#include "drivers/BLDCDriver3PWM.h"
#include "sensors/MagneticSensorI2C.h"
#include "bus/i2c_bus.h"

extern BLDCMotor left_motor;
extern BLDCMotor right_motor;

void motor_init(void);

#endif
