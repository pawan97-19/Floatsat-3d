/*****************************************************************
Sensors.h

Original Created by: Atheel Redah @ University of Würzburg
Original Creation Date: January 20, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4 + Würzburg Uni Informatik 8 Discovery AddOn Board Version 2.0.
*****************************************************************/

#ifndef Sensors_H_
#define Sensors_H_

/* Includes ------------------------------------------------------------------*/
#include "LSM9DS1.h"
#include "math.h"
#include "stdint.h"
#include "topics.h"
#include "stm32f4xx_conf.h"


/* Exported types ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */

void AHRSUpdate();

void GyroUpdate(float gx, float gy, float gz);

void GyroQuaternionUpdate(float gx, float gy, float gz);

void AccMagUpdate(float ax, float ay, float az, float mx, float my, float mz);

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void ExtendedKalmanFilter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void Quaternion2Euler();

void MagCal(float* magMax, float* magMin, float magCalTime);

void GyroCal(float* gbias);

void AccelCal(float* abias);

void MotorSpeedUpdate();

void ADCUpdate();

void EncoderInit();

void multiply_square(double A[7][7], double B[7][7], double C[7][7]);

void multiply_rect1(double A[6][7], double B[7][7], double C[6][7]);

void multiply_rect2(double A[6][7], double B[7][6], double C[6][6]);

void multiply_rect3(double A[7][7], double B[7][6], double C[7][6]);

void multiply_rect4(double A[7][6], double B[6][6], double C[7][6]);

void multiply_rect5(double A[7][6], double B[6], double C[7]);

void multiply_rect6(double A[7][6], double B[6][7], double C[7][7]);

double determinant(double a[6][6], int k);

void cofactor(double num[6][6], int f, double S_cov_inv[6][6]);

void transpose(double num[6][6], double fac[6][6], int r, double S_cov_inv[6][6]);





#endif /* Sensors_H_ */
