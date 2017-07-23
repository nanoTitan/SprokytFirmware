// imu.h

#ifndef _IMU_H_
#define _IMU_H_
	
void IMU_init();
void IMU_update(void);
float IMU_get_sf_yaw();
float IMU_get_sf_pitch();
float IMU_get_sf_roll();
float IMU_get_yaw();
float IMU_get_pitch();
float IMU_get_roll();

#endif // _IMU_H_