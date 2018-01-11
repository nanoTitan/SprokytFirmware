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

/**
 * A header-only class for the Extended Kalman Filter.  Your implementing class should #define the constant N and 
 * and then #include <TinyEKF.h>  You will also need to implement a model() method for your application.
 */

void ekf_init(void *, int, int);
int ekf_step(void *, double *);

typedef struct TinyEKF TinyEKF;
typedef struct TinyEKF_vtbl TinyEKF_vtbl;

struct TinyEKF_vtbl {
	void(*model)(TinyEKF const* self, double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]);
};

struct TinyEKF
{
	ekf_t ekf;
	double * x;
};


/**
* Initializes a TinyEKF object.
*/
void TinyEKF_init(TinyEKF* self) 
{ 
    ekf_init(&self->ekf, Nsta, Mobs); 
	self->x = self->ekf.x; 
}

/**
    * Deallocates memory for a TinyEKF object.
    */
void TinyEKF_deinit(TinyEKF* self) {}

/**
* Implement this function for your EKF model.
* @param fx gets output of state-transition function <i>f(x<sub>0 .. n-1</sub>)</i>
* @param F gets <i>n &times; n</i> Jacobian of <i>f(x)</i>
* @param hx gets output of observation function <i>h(x<sub>0 .. n-1</sub>)</i>
* @param H gets <i>m &times; n</i> Jacobian of <i>h(x)</i>
*/
/*virtual*/ void TinyEKF_model(TinyEKF* self, double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) {}

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

	return ekf_step(&self->ekf, z) ? 0 : 1;
}
