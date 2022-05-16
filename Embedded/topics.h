/*****************************************************************
topics.h

Original Created by: Atheel Redah @ University of Würzburg
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4 + Würzburg Uni Informatik 8 Discovery AddOn Board Version 2.0.
*****************************************************************/

#ifndef __topics_h__
#define __topics_h_

/* Includes ------------------------------------------------------------------*/
#include "hal.h"
#include "math.h"

/* Exported types ------------------------------------------------------------*/

/* Constants*/
const long double PI = 3.141592653589793238L;
const double rad2deg = 180 / PI;

extern long int Rpm_max[3];
extern long int Rpm_min[3];

/* messages transmitted throught the Topics*/
struct Telecommandmsg{
	int32_t DCSpeed[3];
	int32_t STSpeed[3];

	double goal_orientation[6];

	int32_t stopall;

	int32_t valid;

	int32_t estimation;

	int32_t relocation;

	int32_t calibration;

	uint64_t ThreadPeriod[5]; // in Milliseconds

	uint8_t WiFi;

	int32_t SystemMode;

	double Is[3];
	double Irw[3];

	int Filter_mode;
};

struct ImuDatamsg{

	double orientation[6];  // actual orientation of the FLOATSAT

	double euler[3];

	double acceleration[3];  // accelerometer value
	double Gyro[3]; // Gyro value
	double Magneto[3];  // magnetometer value
};

struct APmsg{

	double euler[3];  // actual orientation of the FLOATSAT
};

struct AC2Telemetrie{
// DC motor speed
	int32_t DCSpeed[3];

	// ST motor speed
	int32_t STSpeed[3];

	// moving Masses position
	int32_t Mmpose[3];
};

struct CM2Telemetrie{
// satellite parameters from CM estimation
	double Sat_param[9];
};

struct sPID_Data{
	float Kp, Ki, Kd;
	float P, I, D;
	float e, e_1, Upid, Upid_1, Usat, Usat_1, Kr;
	float Umax, Umin, T;
	bool AntiWindup;
};

struct imudata{
	 int16_t RawDataGx, RawDataGy, RawDataGz; // x, y, and z axis raw data of the gyroscope
	 int16_t RawDataAx, RawDataAy, RawDataAz; // x, y, and z axis raw data of the accelerometer
	 int16_t RawDataMx, RawDataMy, RawDataMz; // x, y, and z axis raw data of the magnetometer
     float gx, gy, gz; // x, y, and z axis readings of the gyroscope in deg/s
     float ax, ay, az; // x, y, and z axis readings of the accelerometer in g
     float mx, my, mz; // x, y, and z axis readings of the magnetometer in Gs
     float temperature; // on-board temperature reading in degree
     float pitch, yaw, roll;
     float q[4];
     float motorSpeed;  // motor speed in RPM
     float motorCurrent;  // motor current in mA
     double deltaTime;
   };

extern Topic<Telecommandmsg> TelecommandDataTopic;
extern Topic<ImuDatamsg> ImuTopic;
extern Topic<APmsg> AttitudeplanningTopic;
extern Topic<AC2Telemetrie> AC2TelemetrieTopic;
extern Topic<CM2Telemetrie> CM2TelemetrieTopic;

/* Methods for the Attitude planning Thread*/

extern void normalize_vect(double *Ar);
extern void cross_prod(double* a, double* b, double* cv);
extern double dot_prod(double* a, double* b);
extern double normvect(double* a);
extern void AxAng2Quat(double* axis, double angle,double* quat);
extern void vect2Quat(double* Ar, double* quat);
extern void hamilton_prod(double* quaternion2, double* quaternion1, double* quaternion12);
extern void matrix_Const_mult(double constant, double* mat, int matlenght, double* result);
extern void matrix_add(double* mat1, double* mat2, int matlenght, double* result);
extern void quat_conj(double* Q, double* result);
extern void quat2eulang(double* quaternion12, double* eulerang);
extern void eul2RotMat(double* eulAng,double* rotmat);

extern void attitudeplanning_Run(double *a, double *b, double* eulang);

extern void motorcontrol_Run(double* posediff, double* dws, float delta_t, double* Is, double* Irw, long int* Rpm);
extern void PID_controller(double* e,double* de,double Kp,double Ki,double Kd,double* ut);

#ifdef __cplusplus
    extern "C"
    {
#endif
    void X_TIM5_IRQHandler(void); /* insert correct prototype */
    void Y_TIM2_IRQHandler(void);
    void Z_TIM5_IRQHandler(void);
        /* add other C function prototypes here if needed */
#ifdef __cplusplus
    }
#endif

#endif

