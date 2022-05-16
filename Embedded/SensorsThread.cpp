/*#
 * Template.cpp
 *
 * Created on: 25.06.2014
 * Author: Atheel Redah
 *
 */
//#include "matlib.h"
#include "rodos.h"
#include <stdio.h>
#include "topics.h"
#include "Sensors.h"
#include "LSM9DS1.h"

// global constants for 9DOF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError M_PI * (400.0f / 180.0f)     // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift M_PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial Quaternion.
// Subsequent changes also require long lag time to a stable output, not fast enough for a fast system!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec.
// This is essentially the I coefficient in a PID control sense; the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 4.0f * 5.0f // This is Kp proportional feedback parameter in the Mahony filter and fusion scheme.
#define Ki 0.0f        // This is Ki integral feedback parameter in the Mahony filter and fusion scheme.

static Application module01("LSM9DS1 AHRS", 2022);

CommBuffer<Telecommandmsg> TelecommandDataBuffer2;
Subscriber TelecommandDataSubscriber2(TelecommandDataTopic, TelecommandDataBuffer2);

// Create an instance of the LSM9DS1 library called `imu`.
LSM9DS1 imu;

HAL_GPIO LSM9DS1_CSAG(GPIO_006); //PA6
HAL_GPIO LSM9DS1_CSM(GPIO_041);  //PC9
HAL_I2C  LSM9DS1_I2C(I2C_IDX2);
HAL_SPI  LSM9DS1_SPI(SPI_IDX1);

AHRS_Mode AHRSModeLast;

uint64_t lastTime = 0;
uint64_t timeNow = 0;
uint64_t startTime = 0;

uint8_t MagCalState=0;

float eInt[3] = {0.0f, 0.0f, 0.0f};       // integral error vector for Mahony method

float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0}, magData[3] = {0, 0, 0};

float magMax[3] = {1000, 1000, 1000}, magMin[3] = {-1000, -1000, -1000};

imudata sensorData = {
		0, 0, 0,	 // int16_t RawDataGx, RawDataGy, RawDataGz
		0, 0, 0,	 // int16_t RawDataAx, RawDataAy, RawDataAz
		0, 0, 0,     // int16_t RawDataMx, RawDataMy, RawDataMz
		0, 0, 0,     // float gx, gy, gz
		0, 0, 0,     // float ax, ay, az
		0, 0, 0,     // float mx, my, mz
		0,           // float temperature
		0, 0, 0,     // float pitch, yaw, roll;
		{1, 0, 0, 0},// float q[4];
		0, 		     // float motorSpeed;
		0            // double deltaTime;
};


class IMUAHRC: public Thread {

public:
	Telecommandmsg TelecommandDataReceiver;
	ImuDatamsg Imudatacollector;
	AHRS_Mode AHRSMode;

public:

	IMUAHRC(const char* name,AHRS_Mode mode) : Thread(name) {
		this->AHRSMode = mode;
	}

	void AHRSUpdate(AHRS_Mode mode)
	{
		switch (mode)
		{

		case Gyro_Cal:
			GyroCal(gbias);
			this->AHRSMode = AHRSModeLast;
			break;

		case Accel_Cal:
			AccelCal(abias);
			this->AHRSMode = AHRSModeLast;
			break;

		case Mag_Cal:
			MagCal(magMax, magMin, 10);
			this->AHRSMode = AHRSModeLast;
			break;

		case Gyro_Update:

			imu.readGyro();    // Read raw gyro data
			sensorData.gx =round(imu.calcGyro(imu.gx - gbias[0]));   // Convert to degrees per seconds, remove gyro biases
			sensorData.gy =round(imu.calcGyro(imu.gy - gbias[1]));
			sensorData.gz =round(imu.calcGyro(imu.gz - gbias[2]));

			timeNow = NOW();
			sensorData.deltaTime = ((timeNow - lastTime) / (double)SECONDS);
			lastTime = timeNow;

			GyroUpdate(sensorData.gx, sensorData.gy, sensorData.gz);
			break;

		case Gyro_Quaternion_Update:

			imu.readGyro();           // Read raw gyro data
			sensorData.gx =round(imu.calcGyro(imu.gx - gbias[0]));   // Convert to degrees per seconds, remove gyro biases
			sensorData.gy =round(imu.calcGyro(imu.gy - gbias[1]));
			sensorData.gz =round(imu.calcGyro(imu.gz - gbias[2]));

			timeNow = NOW();
			sensorData.deltaTime = ((timeNow - lastTime) / (double)SECONDS);
			lastTime = timeNow;

			GyroQuaternionUpdate(sensorData.gx, sensorData.gy, sensorData.gz);
			Quaternion2Euler();
			break;

		case Acc_Mag_Tilted_Compass_Update:

			imu.readAccel();         // Read raw accelerometer data
			sensorData.ax =imu.calcAccel(imu.ax -  abias[0]);   // Convert to g's, remove accelerometer biases
			sensorData.ay =imu.calcAccel(imu.ay -  abias[1]);
			sensorData.az =imu.calcAccel(imu.az -  abias[2]);

			imu.readMag();           // Read raw magnetometer data
			sensorData.mx =imu.calcMag(imu.mx);     // Convert to Gauss
			sensorData.my =imu.calcMag(imu.my);
			sensorData.mz =imu.calcMag(imu.mz);

			magData[0]=(float)(imu.mx-magMin[0])/(float)(magMax[0]-magMin[0])*2-1;
			magData[1]=(float)(imu.my-magMin[1])/(float)(magMax[1]-magMin[1])*2-1;
			magData[2]=(float)(imu.mz-magMin[2])/(float)(magMax[2]-magMin[2])*2-1;

			//The LSM9DS1's magnetometer x and y axes are opposite to the accelerometer, so my and mx are substituted for each other.
			AccMagUpdate((float)(imu.ax -  abias[0]), (float)(imu.ay -  abias[1]), (float)(imu.az -  abias[2]), -magData[1], -magData[0], magData[2]);
			break;

		case Madgwick_Quaternion_Update:

			imu.readGyro();           // Read raw gyro data
			sensorData.gx =round(imu.calcGyro(imu.gx - gbias[0]));   // Convert to degrees per seconds, remove gyro biases
			sensorData.gy =round(imu.calcGyro(imu.gy - gbias[1]));
			sensorData.gz =round(imu.calcGyro(imu.gz - gbias[2]));

			imu.readAccel();         // Read raw accelerometer data
			sensorData.ax =imu.calcAccel(imu.ax -  abias[0]);   // Convert to g's, remove accelerometer biases
			sensorData.ay =imu.calcAccel(imu.ay -  abias[1]);
			sensorData.az =imu.calcAccel(imu.az -  abias[2]);

			imu.readMag();           // Read raw magnetometer data
			sensorData.mx =imu.calcMag(imu.mx);     // Convert to Gauss
			sensorData.my =imu.calcMag(imu.my);
			sensorData.mz =imu.calcMag(imu.mz);

			magData[0]=(float)(imu.mx-magMin[0])/(float)(magMax[0]-magMin[0])*2-1;
			magData[1]=(float)(imu.my-magMin[1])/(float)(magMax[1]-magMin[1])*2-1;
			magData[2]=(float)(imu.mz-magMin[2])/(float)(magMax[2]-magMin[2])*2-1;

			imu.readTemp();
			sensorData.temperature = (float)imu.temperature;

			timeNow = NOW();
			sensorData.deltaTime = ((timeNow - lastTime) / (double)SECONDS);
			lastTime = timeNow;

			//The LSM9DS1's magnetometer x and y axes are opposite to the accelerometer, so my and mx are substituted for each other.
			MadgwickQuaternionUpdate(sensorData.ax, sensorData.ay, sensorData.az, sensorData.gx, sensorData.gy, sensorData.gz, -magData[1], -magData[0], magData[2]);  // Pass gyro rate as rad/s
			Quaternion2Euler();
			break;

		case Mahony_Quaternion_Update:

			imu.readGyro();           // Read raw gyro data
			sensorData.gx =round(imu.calcGyro(imu.gx - gbias[0]));   // Convert to degrees per seconds, remove gyro biases
			sensorData.gy =round(imu.calcGyro(imu.gy - gbias[1]));
			sensorData.gz =round(imu.calcGyro(imu.gz - gbias[2]));

			imu.readAccel();         // Read raw accelerometer data
			sensorData.ax =imu.calcAccel(imu.ax -  abias[0]);   // Convert to g's, remove accelerometer biases
			sensorData.ay =imu.calcAccel(imu.ay -  abias[1]);
			sensorData.az =imu.calcAccel(imu.az -  abias[2]);

			imu.readMag();           // Read raw magnetometer data
			sensorData.mx =imu.calcMag(imu.mx);     // Convert to Gauss
			sensorData.my =imu.calcMag(imu.my);
			sensorData.mz =imu.calcMag(imu.mz);

			magData[0]=(float)(imu.mx-magMin[0])/(float)(magMax[0]-magMin[0])*2-1;
			magData[1]=(float)(imu.my-magMin[1])/(float)(magMax[1]-magMin[1])*2-1;
			magData[2]=(float)(imu.mz-magMin[2])/(float)(magMax[2]-magMin[2])*2-1;

			imu.readTemp();          // Read raw temperature data
			sensorData.temperature = (float)imu.temperature;

			timeNow = NOW();
			sensorData.deltaTime = ((timeNow - lastTime) / (double)SECONDS);
			lastTime = timeNow;

			//The LSM9DS1's magnetometer x and y axes are opposite to the accelerometer, so my and mx are substituted for each other.
			MahonyQuaternionUpdate(sensorData.ax, sensorData.ay, sensorData.az, sensorData.gx, sensorData.gy, sensorData.gz, -magData[1], -magData[0], magData[2]);  // Pass gyro rate as rad/s
			Quaternion2Euler();
			break;
		}
	}

	// This function will calculate your LSM9DS0' orientation angles based on the gyroscope data.
	void GyroUpdate(float gx, float gy, float gz)
	{
		sensorData.pitch = sensorData.pitch + gx * sensorData.deltaTime;
		sensorData.roll  = sensorData.roll  + gy * sensorData.deltaTime;
		sensorData.yaw   = sensorData.yaw   + gz * sensorData.deltaTime;
	}

	// This function will calculate quaternion-based estimation of absolute device orientation using gyroscope data.
	void GyroQuaternionUpdate(float gx, float gy, float gz)
	{
		float q1 = sensorData.q[0], q2 = sensorData.q[1], q3 = sensorData.q[2], q4 = sensorData.q[3];   // short name local variable for readability
		float norm;
		float qDot1, qDot2, qDot3, qDot4;

		gx=gx*M_PI/180.0f;
		gy=gy*M_PI/180.0f;
		gz=gz*M_PI/180.0f;

		// Compute rate of change of quaternion
		qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
		qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy);
		qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx);
		qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx);

		// Integrate to yield quaternion
		q1 += qDot1 * sensorData.deltaTime;
		q2 += qDot2 * sensorData.deltaTime;
		q3 += qDot3 * sensorData.deltaTime;
		q4 += qDot4 * sensorData.deltaTime;

		norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
		norm = 1.0f/norm;

		sensorData.q[0] = q1 * norm;
		sensorData.q[1] = q2 * norm;
		sensorData.q[2] = q3 * norm;
		sensorData.q[3] = q4 * norm;
	}

	void Quaternion2Euler()
	{
		// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
		// In this coordinate system, the positive z-axis is down toward Earth.
		// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination),
		// looking down on the sensor positive yaw is counterclockwise.
		// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
		// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
		// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
		// Tait-Bryan angles as well as Euler angles are non-commutative; that is, to get the correct orientation the rotations must be
		// applied in the correct order which for this configuration is yaw, pitch, and then roll.
		// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
		sensorData.yaw   = atan2(2.0f * (sensorData.q[1] * sensorData.q[2] + sensorData.q[0] * sensorData.q[3]), sensorData.q[0] * sensorData.q[0] + sensorData.q[1] * sensorData.q[1] - sensorData.q[2] * sensorData.q[2] - sensorData.q[3] * sensorData.q[3]);
		sensorData.pitch = -asin(2.0f * (sensorData.q[1] * sensorData.q[3] - sensorData.q[0] * sensorData.q[2]));
		sensorData.roll  = atan2(2.0f * (sensorData.q[0] * sensorData.q[1] + sensorData.q[2] * sensorData.q[3]), sensorData.q[0] * sensorData.q[0] - sensorData.q[1] * sensorData.q[1] - sensorData.q[2] * sensorData.q[2] + sensorData.q[3] * sensorData.q[3]);
		sensorData.pitch*= 180.0f / M_PI;
		sensorData.roll *= 180.0f / M_PI;
		sensorData.yaw  *= 180.0f / M_PI;

		if (sensorData.yaw < 0)
		{
			sensorData.yaw=sensorData.yaw+360;
		}
	}

	// This function will calculate your LSM9DS0' orientation based on accelerometer and magnetometer data.
	void AccMagUpdate(float ax, float ay, float az, float mx, float my, float mz)
	{
		float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch;
		float fTiltedX,fTiltedY;
		float fAcc[3];

		//Rescale the accelerometer radings (in order to increase accuracy in the following norm computation)
		fAcc[0] = ax/100.0; fAcc[1] = ay/100.0; fAcc[2] = az/100.0;

		//Compute the scaled acceleration vector norm
		fNormAcc = sqrt(pow(fAcc[0],2)+pow(fAcc[1],2)+pow(fAcc[2],2));

		//Compute some useful parameters for the g vector rotation matrix
		fSinRoll=fAcc[1]/sqrt(pow(fAcc[1],2)+pow(fAcc[2],2));
		fCosRoll=sqrt(1.0-fSinRoll*fSinRoll);
		fSinPitch=-fAcc[0]/fNormAcc;
		fCosPitch=sqrt(1.0-fSinPitch*fSinPitch);

		//Apply the rotation matrix to the magnetic field vector to obtain the X and Y components on the earth plane
		fTiltedX = mx * fCosPitch + mz * fSinPitch;
		fTiltedY = mx * fSinRoll * fSinPitch + my * fCosRoll - mz * fSinRoll * fCosPitch;


		//return the roll, pitch and heading angles expressed in rad
		sensorData.yaw = -atan2(fTiltedY,fTiltedX);
		sensorData.roll  = atan2(fSinRoll,fCosRoll);
		sensorData.pitch = atan2(fSinPitch,fCosPitch);

		//return the roll, pitch and heading angles expressed in degree
		sensorData.pitch*= 180.0f / M_PI;
		sensorData.roll *= 180.0f / M_PI;
		sensorData.yaw  *= 180.0f / M_PI;
		sensorData.yaw+= 2.22; // Declination at Würzburg, Germany is +2.22 degrees on 27.02.2015.
	}

	// Implementation of Sebastian Madgwick's orientation filter which fuses acceleration, rotation rate, and magnetic moments
	// to produce a quaternion-based estimate of absolute device orientation.
	// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms but is much less computationally intensive.
	void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
	{
		float q1 = sensorData.q[0], q2 = sensorData.q[1], q3 = sensorData.q[2], q4 = sensorData.q[3];   // short name local variable for readability
		float norm;
		float hx, hy, _2bx, _2bz;
		float s1, s2, s3, s4;
		float qDot1, qDot2, qDot3, qDot4;

		// Auxiliary variables to avoid repeated arithmetic
		float _2q1mx;
		float _2q1my;
		float _2q1mz;
		float _2q2mx;
		float _4bx;
		float _4bz;
		float _2q1 = 2.0f * q1;
		float _2q2 = 2.0f * q2;
		float _2q3 = 2.0f * q3;
		float _2q4 = 2.0f * q4;
		float _2q1q3 = 2.0f * q1 * q3;
		float _2q3q4 = 2.0f * q3 * q4;
		float q1q1 = q1 * q1;
		float q1q2 = q1 * q2;
		float q1q3 = q1 * q3;
		float q1q4 = q1 * q4;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q2q4 = q2 * q4;
		float q3q3 = q3 * q3;
		float q3q4 = q3 * q4;
		float q4q4 = q4 * q4;

		gx=gx*M_PI/180.0f;
		gy=gy*M_PI/180.0f;
		gz=gz*M_PI/180.0f;

		// Normalise accelerometer measurement
		norm = sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f/norm;
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Normalise magnetometer measurement
		norm = sqrt(mx * mx + my * my + mz * mz);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f/norm;
		mx *= norm;
		my *= norm;
		mz *= norm;

		// Reference direction of Earth's magnetic field
		_2q1mx = 2.0f * q1 * mx;
		_2q1my = 2.0f * q1 * my;
		_2q1mz = 2.0f * q1 * mz;
		_2q2mx = 2.0f * q2 * mx;
		hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
		hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
		norm = 1.0f/norm;
		s1 *= norm;
		s2 *= norm;
		s3 *= norm;
		s4 *= norm;

		// Compute rate of change of quaternion
		qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
		qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
		qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
		qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

		// Integrate to yield quaternion
		q1 += qDot1 * sensorData.deltaTime;
		q2 += qDot2 * sensorData.deltaTime;
		q3 += qDot3 * sensorData.deltaTime;
		q4 += qDot4 * sensorData.deltaTime;
		norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
		norm = 1.0f/norm;
		sensorData.q[0] = q1 * norm;
		sensorData.q[1] = q2 * norm;
		sensorData.q[2] = q3 * norm;
		sensorData.q[3] = q4 * norm;
	}

	// Mahony filter is similar to Madgwick but uses proportional and integral filtering on the error between estimated reference vectors and measured ones.
	void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
	{
		float q1 = sensorData.q[0], q2 = sensorData.q[1], q3 = sensorData.q[2], q4 = sensorData.q[3];   // short name local variable for readability
		float norm;
		float hx, hy, bx, bz;
		float vx, vy, vz, wx, wy, wz;
		float ex, ey, ez;
		float pa, pb, pc;

		// Auxiliary variables to avoid repeated arithmetic
		float q1q1 = q1 * q1;
		float q1q2 = q1 * q2;
		float q1q3 = q1 * q3;
		float q1q4 = q1 * q4;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q2q4 = q2 * q4;
		float q3q3 = q3 * q3;
		float q3q4 = q3 * q4;
		float q4q4 = q4 * q4;

		gx=gx*M_PI/180.0f;
		gy=gy*M_PI/180.0f;
		gz=gz*M_PI/180.0f;

		// Normalise accelerometer measurement
		norm = sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f / norm;        // use reciprocal for division
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Normalise magnetometer measurement
		norm = sqrt(mx * mx + my * my + mz * mz);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f / norm;        // use reciprocal for division
		mx *= norm;
		my *= norm;
		mz *= norm;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
		hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
		bx = sqrt((hx * hx) + (hy * hy));
		bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

		// Estimated direction of gravity and magnetic field
		vx = 2.0f * (q2q4 - q1q3);
		vy = 2.0f * (q1q2 + q3q4);
		vz = q1q1 - q2q2 - q3q3 + q4q4;
		wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
		wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
		wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

		// Error is cross product between estimated direction and measured direction of gravity
		ex = (ay * vz - az * vy) + (my * wz - mz * wy);
		ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
		ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
		if (Ki > 0.0f)
		{
			eInt[0] += ex;      // accumulate integral error
			eInt[1] += ey;
			eInt[2] += ez;
		}
		else
		{
			eInt[0] = 0.0f;     // prevent integral wind up
			eInt[1] = 0.0f;
			eInt[2] = 0.0f;
		}

		// Apply feedback terms
		gx = gx + Kp * ex + Ki * eInt[0];
		gy = gy + Kp * ey + Ki * eInt[1];
		gz = gz + Kp * ez + Ki * eInt[2];

		// Integrate rate of change of quaternion
		pa = q2;
		pb = q3;
		pc = q4;
		q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * sensorData.deltaTime);
		q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * sensorData.deltaTime);
		q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * sensorData.deltaTime);
		q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * sensorData.deltaTime);

		// Normalise quaternion
		norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
		norm = 1.0f / norm;
		sensorData.q[0] = q1 * norm;
		sensorData.q[1] = q2 * norm;
		sensorData.q[2] = q3 * norm;
		sensorData.q[3] = q4 * norm;
	}

	void MagCal(float* magMax, float* magMin, float magCalTime)
	{
		switch (MagCalState){
		case 0:
			imu.readMag();
			magMax[0]=imu.mx; magMax[1]=imu.my; magMax[2]=imu.mz;
			magMin[0]=imu.mx; magMin[1]=imu.my; magMin[2]=imu.mz;
			PRINTF("Please rotate the magnetometer in all directions around the three axis within %f seconds. \r",magCalTime);
			startTime = NOW();
			MagCalState=1;
			break;
		case 1:
			imu.readMag();
			if (imu.mx > magMax[0]){magMax[0]=imu.mx;}
			else if (imu.mx < magMin[0]){magMin[0]=imu.mx;}

			if (imu.my > magMax[1]){magMax[1]=imu.my;}
			else if (imu.my < magMin[1]){magMin[1]=imu.my;}

			if (imu.mz > magMax[2]){magMax[2]=imu.mz;}
			else if (imu.mz < magMin[2]){magMin[2]=imu.mz;}

			if (((NOW() - startTime) / (double)SECONDS) > magCalTime)
			{
				PRINTF("mMax=[%f %f %f], mMin=[%f %f %f] \r",magMax[0],magMax[1],magMax[2],magMin[0],magMin[1],magMin[2]);
				MagCalState=0;
				AHRSMode=AHRSModeLast;
			}
			break;
		}
	}

	void GyroCal(float* gbias)
	{
		uint8_t data[6] = {0, 0, 0, 0, 0, 0};
		float gyro_bias[3] = {0, 0, 0};
		float samples= 1000;

		PRINTF("Please make sure that the gyroscope is at standstill, the calibration will take about 5 seconds. \r\n");
		AT(NOW()+3000*MILLISECONDS);

		for(int i = 0; i < samples ; i++) {

			imu.readGyro();    // Read raw gyro data
			gyro_bias[0] += imu.gx;
			gyro_bias[1] += imu.gy;
			gyro_bias[2] += imu.gz;
			AT(NOW()+5*MILLISECONDS);
		}

		gbias[0] = round(gyro_bias[0] / samples); // average the data
		gbias[1] = round(gyro_bias[1] / samples);
		gbias[2] = round(gyro_bias[2] / samples);

		PRINTF("gxbias = %f, gybias = %f, gzbias = %f \r\n", gbias[0], gbias[1], gbias[2]);
	}

	void AccelCal(float* abias)
	{
		uint8_t data[6] = {0, 0, 0, 0, 0, 0};
		float accel_bias[3] = {0, 0, 0};
		float samples= 1000;

		PRINTF("Please make sure that the accelerometer is standstill at x=0g, y=0g, z=1g position, the calibration will take about 5 seconds. \r\n");
		AT(NOW()+3000*MILLISECONDS);

		for(int i = 0; i < samples ; i++) {

			imu.readAccel();    // Read raw Accel data
			accel_bias[0] += imu.ax;
			accel_bias[1] += imu.ay;
			////accel_bias[2] += imu.az; // Assumes sensor facing up!;
			AT(NOW()+5*MILLISECONDS);
		}

		abias[0] = round(accel_bias[0] / samples); // average the data
		abias[1] = round(accel_bias[1] / samples);
		abias[2] = round(accel_bias[2] / samples);


		PRINTF("axbias = %f, aybias = %f, azbias = %f \r\n", abias[0], abias[1], abias[2]);
	}

	void init() {
	}

	void run() {

		AT(NOW()+4*MILLISECONDS);
		uint64_t thread_period = 50;
		this->TelecommandDataReceiver.Filter_mode = 8;
		// this->TelecommandDataReceiver.ThreadPeriod[2] = -1;

		// note, we need to sent this our CS pins (defined above)
		if (imu.beginSPI() == false)
		{
			PRINTF("Failed to communicate with LSM9DS1.\r\n");
		}

		GyroCal(gbias);
		AT(NOW()+4000*MILLISECONDS);
		AccelCal(abias);
		AT(NOW()+4000*MILLISECONDS);

		TIME_LOOP(0,thread_period){

			TelecommandDataBuffer2.getOnlyIfNewData(this->TelecommandDataReceiver);

			thread_period = this->TelecommandDataReceiver.ThreadPeriod[2] == -1? thread_period:this->TelecommandDataReceiver.ThreadPeriod[2];
			this->AHRSMode = (AHRS_Mode)this->TelecommandDataReceiver.Filter_mode;

/*
			PRINTF("The assigned value to the filter Mode is = %d \r\n", this->TelecommandDataReceiver.Filter_mode); // in degree
			PRINTF("The actual value of the filter Mode is = %d \r\n", this->AHRSMode); // in degree
*/

			if(this->TelecommandDataReceiver.calibration ==1 && this->TelecommandDataReceiver.SystemMode == 0){
				AHRSUpdate(Gyro_Cal);
				AT(NOW()+2000*MILLISECONDS);
				AHRSUpdate(Accel_Cal);
				AT(NOW()+2000*MILLISECONDS);
				AHRSUpdate(Mag_Cal);
				AT(NOW()+2000*MILLISECONDS);
			}else if(this->TelecommandDataReceiver.calibration ==2 && this->TelecommandDataReceiver.SystemMode == 0){
				AHRSUpdate(Gyro_Cal);
			}else if(this->TelecommandDataReceiver.calibration ==3 && this->TelecommandDataReceiver.SystemMode == 0){
				AHRSUpdate(Accel_Cal);
			}else if(this->TelecommandDataReceiver.calibration ==4 && this->TelecommandDataReceiver.SystemMode == 0){
				AHRSUpdate(Mag_Cal);
			}

			AHRSUpdate(this->AHRSMode);

			this->Imudatacollector.acceleration[0] = sensorData.ax;
			this->Imudatacollector.acceleration[1] = sensorData.ay;
			this->Imudatacollector.acceleration[2] = sensorData.az;

			this->Imudatacollector.Gyro[0] = sensorData.gx;
			this->Imudatacollector.Gyro[1] = sensorData.gy;
			this->Imudatacollector.Gyro[2] = sensorData.gz;

			this->Imudatacollector.Magneto[0] = sensorData.mx;
			this->Imudatacollector.Magneto[1] = sensorData.my;
			this->Imudatacollector.Magneto[2] = sensorData.mz;

			this->Imudatacollector.orientation[0] = sensorData.pitch*PI/180;
			this->Imudatacollector.orientation[1] = sensorData.roll*PI/180;
			this->Imudatacollector.orientation[2] = sensorData.yaw*PI/180;

			PRINTF("Pitch = %f, Roll = %f, Yaw = %f \r\n", sensorData.pitch, sensorData.roll, sensorData.yaw); // in degree

			ImuTopic.publish(this->Imudatacollector);
		}
	}
};
IMUAHRC IMUAHRC("IMUAHRC",Mahony_Quaternion_Update);

/***********************************************************************/
