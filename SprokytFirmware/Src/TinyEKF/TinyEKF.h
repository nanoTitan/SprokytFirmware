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
	double P_imu = 1;
	double P_dd = 1;
	double R_imu = 12.96;		// 3.6 degree or 1% accuracy (standard dev) in IMU. variance = 3.6^2
	double R_dd = 4;		// 2 degree or 0.556% accuracy (standard dev)  in Diff Drive. variance = 2^2
	
	int i;
	
//	self->ekf.P[0][0] = P_imu;
//	self->ekf.P[1][1] = P_dd;
	
	// Set the initial states
	for (i = 0; i < Nsta; ++i)
		self->ekf.x[i] = 0;
	
	// Approximate the process noise using a small constant
	TinyEKF_setQ(self, 0, 0, .0001);

	// Approximate the measurement noise
	TinyEKF_setR(self, 0, 0, R_imu);
	TinyEKF_setR(self, 1, 1, R_dd);
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
	// Process model is f(x) = x
	fx[0] = self->x[0];

	// process model Jacobian is identity matrix
	F[0][0] = 1;

	// Measurement function simplifies the relationship between state and sensor readings for convenience.
	// A more realistic measurement function would distinguish between state value and measured value; e.g.:
	//   hx[0] = pow(this->x[0], 1.03);
	//   hx[1] = 1.005 * this->x[1];
	//   hx[2] = .9987 * this->x[1] + .001;
	hx[0] = self->x[0]; // IMU heading from previous state
	hx[1] = self->x[0]; // Diff Drive heading from previous state

	// process model Jacobian is identity matrix
	H[0][0] = 1;
	H[1][0] = 1;
}

/**
* Sets the specified value of the prediction error covariance. <i>P<sub>i,j</sub> = value</i>
* @param i row index
* @param j column index
* @param value value to set
*/
void TinyEKF_setP(TinyEKF* self, int i, int j, double value) 
{ 
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
    return self->ekf.x[i]; 
}

/**
* Sets the state element at a given index.
* @param i the index (at least 0 and less than <i>n</i>
* @param value value to set
*/
void TinyEKF_setX(TinyEKF* self, int i, double value) 
{ 
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