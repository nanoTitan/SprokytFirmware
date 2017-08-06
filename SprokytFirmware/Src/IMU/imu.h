#pragma once
	
typedef void(*ImuFunctionCallback)(float[], int size);

void IMU_init();
void IMU_update(void);
float IMU_get_yaw();
float IMU_get_pitch();
float IMU_get_roll();
void RegisterImuCallback(ImuFunctionCallback callback);
