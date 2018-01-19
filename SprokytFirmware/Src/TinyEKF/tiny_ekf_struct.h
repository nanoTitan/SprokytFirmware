/*
 * tiny_ekf_struct.h: common data structure for TinyEKF
 *
 * You should #include this file after using #define for N (states) and M
*  (observations)
 *
 * Copyright (C) 2016 Simon D. Levy
 *
 * MIT License
 */

#include "constants.h"

typedef struct {

    int n;          /* number of state values */
    int m;          /* number of observables */

    double x[Nsta];    /* state vector													x_k	*/

    double P[Nsta][Nsta];  /* prediction error covariance */
    double Q[Nsta][Nsta];  /* process noise covariance									w_k */
    double R[Mobs][Mobs];  /* measurement error covariance */

    double G[Nsta][Mobs];  /* Kalman gain; a.k.a. K */

    double F[Nsta][Nsta];  /* Jacobian of process model */
    double H[Mobs][Nsta];  /* Jacobian of measurement model */

    double Ht[Nsta][Mobs]; /* transpose of measurement Jacobian */
    double Ft[Nsta][Nsta]; /* transpose of process Jacobian */
    double Pp[Nsta][Nsta]; /* P, post-prediction, pre-update							P_k	*/

    double fx[Nsta];   /* output of user defined f() state-transition function			fx	*/
    double hx[Mobs];   /* output of user defined h() measurement function				hx	*/

    /* temporary storage */
    double tmp0[Nsta][Nsta];
    double tmp1[Nsta][Mobs];
    double tmp2[Mobs][Nsta];
    double tmp3[Mobs][Mobs];
    double tmp4[Mobs][Mobs];
    double tmp5[Mobs]; 

} ekf_t;        
