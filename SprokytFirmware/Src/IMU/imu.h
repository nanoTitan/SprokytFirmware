#pragma once

#include "constants.h"

void IMU_init();
void IMU_update(void);
float IMU_get_yaw();
float IMU_get_pitch();
float IMU_get_roll();
void IMU_RegisterAngularPosCallback(AngularPositionCallback callback);
