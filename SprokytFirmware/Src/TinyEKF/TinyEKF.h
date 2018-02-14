/*
* TinyEKF: Extended Kalman Filter for Arduino and TeensyBoard.
*
* Copyright (C) 2015 Simon D. Levy
*
* MIT License
*/

#include <stdio.h>
#include <stdlib.h>
#include "tiny_ekf_struct.h"
#include "debug.h"
#include "constants.h"
#include <assert.h>

/**
 * A header-only class for the Extended Kalman Filter.  Your implementing class should #define the constant N and 
 * and then #include <TinyEKF.h>  You will also need to implement a model() method for your application.
 */

typedef struct TinyEKF TinyEKF;
typedef struct TinyEKF_vtbl TinyEKF_vtbl;

void ekf_init(void *, int, int);
int ekf_step(void *, double *);
void TinyEKF_print(TinyEKF* self);
void TinyEKF_init_local(TinyEKF* self);
void TinyEKF_model(TinyEKF* self, double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]);
void TinyEKF_setP(TinyEKF* self, int i, int j, double value);
void TinyEKF_setQ(TinyEKF* self, int i, int j, double value);
void TinyEKF_setR(TinyEKF* self, int i, int j, double value);
double TinyEKF_getX(TinyEKF* self, int i);
void TinyEKF_setX(TinyEKF* self, int i, double value);
uint8_t TinyEKF_step(TinyEKF* self, double * z);


struct TinyEKF_vtbl {
	void(*model)(TinyEKF const* self, double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]);
};

struct TinyEKF
{
	ekf_t ekf;
	double* x;
};


/**
* Initializes a TinyEKF object.
*/
void TinyEKF_init(TinyEKF* self) 
{ 
    ekf_init(&self->ekf, Nsta, Mobs); 	
	
	TinyEKF_init_local(self);
	self->x = self->ekf.x;
}

/**
* Initialize errors, initial states and Jacobian functions
* TODO: Move this function into an instance initialization action
*/
void TinyEKF_init_local(TinyEKF* self)
{
	// initial covariances of state noise, measurement noise
	double R_uwb_x = .1;		// TODO: UWB error (variance) of 10cm or .01^2
	double R_uwb_z =.1;			// TODO: UWB error (variance) of 10cm or .01^2
	double R_dd_x =.1;			// TODO: DD error (variance) of 10cm or .1^2
	double R_dd_z =.1;			// TODO: DD error (variance) of 10cm or .1^2
	double R_imu_yaw = 1.296;	// 3.6 degree or 1% accuracy (standard dev) in IMU. variance = 3.6^2
	double R_dd_yaw = 0.4;		// 2 degree or 0.556% accuracy (standard dev)  in Diff Drive. variance = 2^2
	
	int i;
	
	// Set the initial states
	for (i = 0; i < Nsta; ++i)
		self->ekf.x[i] = 0;
	
	// Approximate the process noise using a small constant
	TinyEKF_setQ(self, 0, 0, .0001);
	TinyEKF_setQ(self, 1, 1, .0001);
	TinyEKF_setQ(self, 2, 2, .0001);

	// Approximate the measurement noise
	TinyEKF_setR(self, 0, 0, R_uwb_x);
	TinyEKF_setR(self, 1, 1, R_dd_x);
	TinyEKF_setR(self, 2, 2, R_uwb_z);
	TinyEKF_setR(self, 3, 3, R_dd_z);
	TinyEKF_setR(self, 4, 4, R_imu_yaw);
	TinyEKF_setR(self, 5, 5, R_dd_yaw);
	
	//TinyEKF_print(self);
}

/**
* Implement this function for your EKF model.
* @param fx gets output of state-transition function <i>f(x<sub>0 .. n-1</sub>)</i>
* @param F gets <i>n &times; n</i> Jacobian of <i>f(x)</i>
* @param hx gets output of observation function <i>h(x<sub>0 .. n-1</sub>)</i>
* @param H gets <i>m &times; n</i> Jacobian of <i>h(x)</i>
*/
/*virtual*/ void TinyEKF_model(TinyEKF* self, double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) 
{
	// state transition model is x_k = f(x_k-1, u_k)
	fx[0] = self->x[0];
	fx[1] = self->x[1];
	fx[2] = self->x[2];

	// process model Jacobian is identity matrix
	F[0][0] = 1;
	F[1][1] = 1;
	F[2][2] = 1;
	
	// Measurement function simplifies the relationship between state and sensor readings for convenience.
	// A more realistic measurement function would distinguish between state value and measured value; e.g.:
	//   hx[0] = pow(this->x[0], 1.03);
	//   hx[1] = 1.005 * this->x[1];
	//   hx[2] = .9987 * this->x[1] + .001;
	hx[0] = self->x[0]; // uwb x from previous state
	hx[1] = self->x[0]; // dd x from previous state
	hx[2] = self->x[1]; // uwb z from previous state
	hx[3] = self->x[1]; // dd z from previous state
	hx[4] = self->x[2]; // IMU heading from previous state
	hx[5] = self->x[2]; // Diff Drive heading from previous state

	// process model Jacobian is identity matrix
	H[0][0] = 1;
	H[1][0] = 1;
	H[2][1] = 1;
	H[3][1] = 1;
	H[4][2] = 1;
	H[5][2] = 1;
}

/**
* Sets the specified value of the prediction error covariance. <i>P<sub>i,j</sub> = value</i>
* @param i row index
* @param j column index
* @param value value to set
*/
void TinyEKF_setP(TinyEKF* self, int i, int j, double value) 
{ 
	assert(i < Nsta && i > -1 && j < Nsta && j > -1);
	self->ekf.P[i][j] = value; 
}

/**
* Sets the specified value of the process noise covariance. <i>Q<sub>i,j</sub> = value</i>
* @param i row index
* @param j column index
* @param value value to set
*/
void TinyEKF_setQ(TinyEKF* self, int i, int j, double value) 
{ 
	assert(i < Nsta && i > -1 && j < Nsta && j > -1);
	self->ekf.Q[i][j] = value; 
}

/**
* Sets the specified value of the observation noise covariance. <i>R<sub>i,j</sub> = value</i>
* @param i row index
* @param j column index
* @param value value to set
*/
void TinyEKF_setR(TinyEKF* self, int i, int j, double value) 
{ 
	assert(i < Mobs && i > -1 && j < Mobs && j > -1);
	self->ekf.R[i][j] = value; 
}

// public

/**
* Returns the state element at a given index.
* @param i the index (at least 0 and less than <i>n</i>
* @return state value at index
*/
double TinyEKF_getX(TinyEKF* self, int i) 
{ 
	assert(i < Nsta && i > -1);
    return self->ekf.x[i]; 
}

/**
* Sets the state element at a given index.
* @param i the index (at least 0 and less than <i>n</i>
* @param value value to set
*/
void TinyEKF_setX(TinyEKF* self, int i, double value) 
{ 
	assert(i < Nsta && i > -1);
	self->ekf.x[i] = value; 
}

/**
Performs one step of the prediction and update.
* @param z observation vector, length <i>m</i>
* @return true on success, false on failure caused by non-positive-definite matrix.
*/
uint8_t TinyEKF_step(TinyEKF* self, double * z) 
{ 
	TinyEKF_model(self, self->ekf.fx, self->ekf.F, self->ekf.hx, self->ekf.H); 
	//TinyEKF_print(self);

	int result = ekf_step(&self->ekf, z) ? 0 : 1;
	return result;
}

void TinyEKF_print(TinyEKF* self)
{
	ekf_t* ekf = &self->ekf;
	int i, j;
	double* curr;
	PRINTF("m: %d, n: %d\n", ekf->m, ekf->n);
	
	PRINTF("\nx: ");
	for (i = 0; i < Nsta; ++i)
		PRINTF("%f, ", ekf->x[i]);
	
	PRINTF("\nP: ");
	for (i = 0; i < Nsta; ++i)
		for (j = 0; j < Nsta; ++j)
			PRINTF("%f, ", ekf->P[i][j]);
	
	PRINTF("\nQ: ");
	for (i = 0; i < Nsta; ++i)
		for (j = 0; j < Nsta; ++j)
			PRINTF("%f, ", **(ekf->Q + i));
	
	PRINTF("\nR: ");
	for (i = 0; i < Mobs; ++i)
		for (j = 0; j < Mobs; ++j)
			PRINTF("%f, ", ekf->R[i][j]);
	
	PRINTF("\nG: ");
	for (i = 0; i < Nsta; ++i)
		for (j = 0; j < Mobs; ++j)
			PRINTF("%f, ", ekf->G[i][j]);
	
	PRINTF("\nF: ");
	for (i = 0; i < Nsta; ++i)
		for (j = 0; j < Nsta; ++j)
			PRINTF("%f, ", ekf->F[i][j]);
	
	PRINTF("\nH: ");
	for (i = 0; i < Mobs; ++i)
		for (j = 0; j < Nsta; ++j)
			PRINTF("%f, ", ekf->H[i][j]);
	
	PRINTF("\nHt: ");
	for (i = 0; i < Nsta; ++i)
		for (j = 0; j < Mobs; ++j)
			PRINTF("%f, ", ekf->Ht[i][j]);
	
	PRINTF("\nFt: ");
	for (i = 0; i < Nsta; ++i)
		for (j = 0; j < Nsta; ++j)
			PRINTF("%f, ", ekf->Ft[i][j]);
	
	PRINTF("\nPp: ");
	for (i = 0; i < Nsta; ++i)
		for (j = 0; j < Nsta; ++j)
			PRINTF("%f, ", ekf->Pp[i][j]);
	
	PRINTF("\nfx: ");
	for (i = 0; i < Nsta; ++i)
		PRINTF("%f, ", ekf->fx[i]);
	
	PRINTF("\nhx: ");
	for (i = 0; i < Mobs; ++i)
		PRINTF("%f, ", ekf->hx[i]);
	
	PRINTF("\n\n");
}