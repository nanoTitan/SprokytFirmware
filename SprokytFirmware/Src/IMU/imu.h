#pragma once

#include "constants.h"

void IMU_init();
void IMU_update(void);
uint8_t IMU_get_sensorFusionStable();
float IMU_get_yaw();
float IMU_get_pitch();
float IMU_get_roll();
void IMU_get_yawPitchRoll(float* out_yaw, float* out_pitch, float* out_roll);
void IMU_RegisterAngularPosCallback(AngularPositionCallback callback);
