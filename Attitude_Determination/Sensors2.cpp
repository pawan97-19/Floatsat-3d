/*
 * Template.cpp
 *
 * Created on: 25.06.2014
 * Author: Atheel Redah
 *
 */

#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "math.h"
#include "stm32f4xx_conf.h"

#include "LSM9DS1.h"
#include "math.h"
#include "Sensors.h"



CommBuffer<sTelecommandData> SensorsTelecommandDataBuffer;
Subscriber SensorsTelecommandDataSubscriber(TelecommandDataTopic, SensorsTelecommandDataBuffer);

sSensorData sensorData = {0, 0, 0,	   // int16_t RawDataGx, RawDataGy, RawDataGz
						  0, 0, 0,	   // int16_t RawDataAx, RawDataAy, RawDataAz
						  0, 0, 0,     // int16_t RawDataMx, RawDataMy, RawDataMz
						  0, 0, 0,     // float gx, gy, gz
						  0, 0, 0,     // float ax, ay, az
                          0, 0, 0,     // float mx, my, mz
                          0,           // float temperature
						  0, 0, 0,     // float pitch, yaw, roll;
						  {1, 0, 0, 0},// float q[4];
						  0, 		   // float motorSpeed;
						  0            // double deltaTime;
};

/* Private variables ---------------------------------------------------------*/
__IO uint32_t IC4ReadValue1 = 0, IC4ReadValue2 = 0, Capture = 0;
__IO uint8_t CaptureNumber = 0;
__IO uint32_t TIM2Freq = 0;
__IO uint8_t EncoderB;
__IO double CaptureTime;

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

HAL_GPIO LSM9DS1_CSAG(GPIO_006); //PA6
HAL_GPIO LSM9DS1_CSM(GPIO_041);  //PC9
HAL_I2C  LSM9DS1_I2C(I2C_IDX2);
HAL_SPI  LSM9DS1_SPI(SPI_IDX1);

#define GyroMeasError M_PI * (400.0f / 180.0f)     // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift M_PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 3.21 // Declination (degrees) in Würzburg
double x_inv[36] = {0};
double y[36] = {0};
float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0}, magData[3] = {0, 0, 0};

float magMax[3] = {1000, 1000, 1000}, magMin[3] = {-1000, -1000, -1000};




uint64_t SensorsPeriod = 1000; // Sensors period in ms

uint64_t lastTime = 0;
uint64_t timeNow = 0;
uint64_t startTime = 0;

uint8_t MagCalState=0;


//Function definitions
void printGyro();
void printAccel();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);

void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  imu.readGyro();

  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  PRINTF("gx=%f, gy=%f, gz=%f \r\n",imu.calcGyro(imu.gx),imu.calcGyro(imu.gy),imu.calcGyro(imu.gz));
  //PRINTF("Gx=%d, Gy=%d, Gz=%d \r\n",imu.gx,imu.gy,imu.gz);
}

void printAccel()
{
    // To read from the accelerometer, you must first call the
   // readAccel() function. When this exits, it'll update the
   // ax, ay, and az variables with the most current data.
	imu.readAccel();

   // Now we can use the ax, ay, and az variables as we please.
    // Either print them as raw ADC values, or calculated in g's.
  PRINTF("ax=%f, ay=%f, az=%f \r\n",imu.calcAccel(imu.ax),imu.calcAccel(imu.ay),imu.calcAccel(imu.az));
  //PRINTF("Ax=%d, Ay=%d, Az=%d \r\n",imu.ax,imu.ay,imu.az);
}

void printMag()
{
	 // To read from the magnetometer, you must first call the
	  // readMag() function. When this exits, it'll update the
	  // mx, my, and mz variables with the most current data.
	  imu.readMag();

	  // Now we can use the mx, my, and mz variables as we please.
	  // Either print them as raw ADC values, or calculated in Gauss.
  PRINTF("mx=%f, my=%f, mz=%f \r\n",imu.calcAccel(imu.mx),imu.calcAccel(imu.my),imu.calcAccel(imu.mz));
  //PRINTF("Mx=%d, My=%d, Mz=%d \r\n",imu.mx,imu.my,imu.mz);
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, sqrt(ax * ax + az * az));
  float pitch = atan2(ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? M_PI : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * M_PI / 180;

  if (heading > M_PI) heading -= (2 * M_PI);
  else if (heading < -M_PI) heading += (2 * M_PI);

  // Convert everything from radians to degrees:
  heading *= 180.0 / M_PI;
  pitch *= 180.0 / M_PI;
  roll  *= 180.0 / M_PI;

  PRINTF("Pitch=%f, Roll=%f, Heading=%f \r\n",pitch,roll,heading);

}


void GyroCal(float* gbias)
{
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  float gyro_bias[3] = {0, 0, 0};
  float samples= 1000;

  PRINTF("Please make sure that the gyroscope is at standstill, the calibration will take about 5 seconds. \r\n");

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

		}
		break;
	}
}

void AccelCal(float* abias)
{
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  float accel_bias[3] = {0, 0, 0};
  float samples= 1000;

  PRINTF("Please make sure that the accelerometer is standstill at x=0g, y=0g, z=1g position, the calibration will take about 5 seconds. \r\n");

  for(int i = 0; i < samples ; i++) {

	imu.readAccel();    // Read raw Accel data
	accel_bias[0] += imu.ax;
	accel_bias[1] += imu.ay;
	accel_bias[2] += imu.az; // Assumes sensor facing up!;
    AT(NOW()+5*MILLISECONDS);
  }

  abias[0] = round(accel_bias[0] / samples); // average the data
  abias[1] = round(accel_bias[1] / samples);
  abias[2] = round(accel_bias[2] / samples);

  PRINTF("axbias = %f, aybias = %f, azbias = %f \r\n", abias[0], abias[1], abias[2]);
}

void AHRSUpdate()
{


	printGyro();           // Read raw gyro data
			sensorData.gx =round(imu.calcGyro(imu.gx - gbias[0]));   // Convert to degrees per seconds, remove gyro biases
			sensorData.gy =round(imu.calcGyro(imu.gy - gbias[1]));
			sensorData.gz =round(imu.calcGyro(imu.gz - gbias[2]));
PRINTF("I am working");
			printAccel(); // Print "A: ax, ay, az"         // Read raw accelerometer data
			sensorData.ax =imu.calcAccel(imu.ax -  abias[0]);   // Convert to g's, remove accelerometer biases
			sensorData.ay =imu.calcAccel(imu.ay -  abias[1]);
			sensorData.az =imu.calcAccel(imu.az -  abias[2]);

			printMag();         // Read raw magnetometer data
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
			ExtendedKalmanFilter(sensorData.ax, sensorData.ay, sensorData.az, sensorData.gx, sensorData.gy, sensorData.gz, -magData[1], -magData[0], magData[2]);  // Pass gyro rate as rad/s
			Quaternion2Euler();
			PRINTF("x = %f  y = %f  z = %f", sensorData.roll,sensorData.pitch, sensorData.yaw );


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


void ExtendedKalmanFilter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float X_state[7] = {sensorData.q[0], sensorData.q[1], sensorData.q[2], sensorData.q[3], gbias[0], gbias[1], gbias[2]};
    float q0 = sensorData.q[0], q1 = sensorData.q[1], q2 = sensorData.q[2], q3 = sensorData.q[3];

    double H[6][7];
    float dt = sensorData.deltaTime;
    double F[7][7] = {0};
    double F_trans[7][7] = {0};
    double P_local[7][7] = {0};
    double P_local_temp[7][7] = {0};
    double P_temp[7][7] = {0};
    double P_new[7][7] = {0};
    double y_res[5] = {0};
    float h_mes[5] = {0};
    float P_extern[7][7]={
                             {1000, 0, 0, 0, 0, 0, 0},
                             {0, 1000, 0, 0, 0, 0, 0},
                             {0, 0, 1000, 0, 0, 0, 0},
                             {0, 0, 0, 1000, 0, 0, 0},
                             {0, 0, 0,    0, 1, 0, 0},
                             {0, 0, 0,    0, 0, 1, 0},
                             {0, 0, 0,    0, 0, 0, 1},
                             };

    double S_cov[6][6] = {0};
    double S_Temp[6][6] = {0};
    double S_cov_inv[6][6] = {0};
    double K_Gain[7][6] = {0};
    double K_Temp[7][6] = {0};
    double Del_x[7] = {0};
    double Q[7]={0,0,0,0,0.0000008,0.0000008,0.0000008};
    double R[6]={100.765,100.765,100.765,10000.45,10000.45,10000.45};
    float norm;
    float hx, hy, bx, bz;
    int i, j;

	PRINTF("I am working33 ");





//Quaternion state propogation
    q0 = q0 + dt*0.5f*(-q1*(gx-gbias[0]) - q2*(gy-gbias[1]) - q3*(gz - gbias[2]));
    q1 = q0 + dt*0.5f*(q0*(gx-gbias[0]) - q3*(gy-gbias[1]) + q2*(gz - gbias[2]));
    q2 = q0 + dt*0.5f*(q3*(gx-gbias[0]) + q0*(gy-gbias[1]) - q1*(gz - gbias[2]));
    q3 = q0 + dt*0.5f*(-q2*(gx-gbias[0]) + q1*(gy-gbias[1]) + q0*(gz - gbias[2]));



// Normalize Quaternion
float qnorm = (double)sqrt(((q0*q0+q1*q1)+q2*q2)+q3*q3);
q0*=1/qnorm;
q1*=1/qnorm;
q2*=1/qnorm;
q3*=1/qnorm;

X_state[0] = q0;
X_state[1] = q1;
X_state[2] = q2;
X_state[3] = q3;

// Jacobian for state function */
F[0][0] = 1.0f;
F[0][1] = -(dt*0.5f)*(gx - gbias[0]);
F[0][2] = -(dt * 0.5f ) * (gy - gbias[1]);
F[0][3] = -(dt * 0.5f ) * (gz - gbias[2]) ;
F[0][4] = 0.5f*q1*dt;
F[0][5] = 0.5f*q2*dt;
F[0][6] = 0.5f*q3*dt;

F[1][0] = (dt * 0.5f )*(gx - gbias[0]);
F[1][1] = 1.0f;
F[1][2] = (dt * 0.5f ) * (gz - gbias[2]);
F[1][3] = -(dt * 0.5f ) * (gy - gbias[1]);
F[1][4] = -0.5f*q0* dt ;
F[1][5] = 0.5f*q3*dt;
F[1][6] = -0.5f*q2*dt;

F[2][0] = (dt * 0.5f) * (gy - gbias[1]);
F[2][0] = -(dt * 0.5F) * (gz - gbias[2]);
F[2][2] = 1;
F[2][3] = (dt * 0.5F) * (gx - gbias[0]);
F[2][4] = -0.5f*q3*dt;
F[2][5] = -0.5f*q0*dt;
F[2][6] = 0.5f*q1*dt;

F[3][0] = (dt * 0.5F) * (gz - gbias[2]);
F[3][1] = (dt * 0.5F) * (gy - gbias[1]);
F[3][2] = -(dt * 0.5F) * (gx - gbias[0]);
F[3][3] = 1;
F[3][4] = 0.5F*q2*dt;
F[3][5] = -0.5f*q1*dt;
F[3][6] = -0.5f*q0*dt;

F[4][4] = 1;
F[5][5] = 1;
F[6][6] = 1;

//* predicted covariance estimate */

for(i = 0; i<7; i++){
    for(j = 0; j<7; j++){
        F_trans[i][j] = F[j][i];
        P_local[i][j] = P_extern[i][j];
        }
    }

multiply_square(F, P_local, P_local_temp);
multiply_square(P_local_temp, F_trans, P_local);

for (i = 0; i<7; i++){
    P_local[i][i] = P_local[i][i]+Q[i] ;
    }


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
                    hx = 2.0f * mx * (0.5f - q2*q2 - q3*q3) + 2.0f * my * (q1*q2 -q1*q3) + 2.0f * mz * (q1*q3 + q0*q2);
                    hy = 2.0f * mx * (q1*q2 + q0*q3) + 2.0f * my * (0.5f - q1*q1 - q3*q3) + 2.0f * mz * (q2*q3 - q0*q1);
                    bx = sqrt((hx * hx) + (hy * hy));
                    bz = 2.0f * mx * (q1*q3 - q0*q2) + 2.0f * my * (q2*q3 + q0*q1) + 2.0f * mz * (0.5f - q1*q1 - q2*q2);

   // Creating Measurement residual y = Z - h(x) */
   float Z[5] = {0};
   Z[0] = ax;
   Z[1] = ay;
   Z[2] = az;
   Z[3] = mx;
   Z[4] = my;
   Z[5] = mz;

   h_mes[0] = -2 * (q1*q3-q0*q2);
   h_mes[1] = -2 * (q2*q3+q0*q1);
   h_mes[2] = -(1-2*q1*q1-2*q2*q2);
   h_mes[3] = bx * (1-2*q2*q2-2*q3*q3)+2*bz*(q1*q3-q0*q2);
   h_mes[4] = 2 * bx * (q1*q2-q0*q3)+2*bz*(q2*q3+q0*q1);
   h_mes[5] = 2 * bx * (q1*q3+q0*q2)+bz*(1-2*q1*q1-2*q2*q2);

   for(i = 0; i<6; i++){
	   y_res[i] = Z[i] - h_mes[i];
   }


  //H (measurement jacobian)


double H_trans[7][6] = {0};

    H[0][0] = 2*q2;
    H[0][1] = -2*q3;
    H[0][2] = 2*q0;
    H[0][3] = -2*q1;
    H[0][4] = 0;
    H[0][5] = 0;
    H[0][6] = 0;

    H[1][0] = -2*q1;
    H[1][1] = -2*q0;
    H[1][2] = -2*q3;
    H[1][3] = -2*q2;
    H[1][4] = 0;
    H[1][5] = 0;
    H[1][6] = 0;

    H[2][0] = 0;
    H[2][1] = 4*q1;
    H[2][2] = 4*q2;
    H[2][3] = 0;
    H[2][4] = 0;
    H[2][5] = 0;
    H[2][6] = 0;

    H[3][0] = -2*bz*q2;
    H[3][1] = 2*bz*q3;
    H[3][2] = -4*q2* bx-2*bz*q0;
    H[3][3] = -4*bx *q3 + 2*bz *q1;
    H[3][4] = 0;
    H[3][5] = 0;
    H[3][6] = 0;

    H[4][0] = -2*bx *q3 + 2*bz *q1;
    H[4][1] = 2*bx *q2 + 2*bz *q0;
    H[4][2] = 2*bx *q1 + 2*bz *q3;
    H[4][3] = -2*bx *q0 + 2*bz *q2;
    H[4][4] = 0;
    H[4][5] = 0;
    H[4][6] = 0;

    H[5][0] = 2*bx *q2;
    H[5][1] = 2*bx *q3 - 4*q1* bz ;
    H[5][2] = 2*bx *q0 - 4*bz *q2 ;
    H[5][3] = 2*bx *q1 ;
    H[5][4] = 0;
    H[5][5] = 0;
    H[5][6] = 0;

    for(i = 0; i<7; i++){
    	for(j=0; j<6; j++){
    		H_trans[i][j] = H[j][i];
    	}
    }



    //* Residual Covaraince S = H*P*H_trans + R*/

    multiply_rect1(H, P_local, P_local_temp);
    multiply_rect2(P_local_temp, H_trans, S_cov);

    for(i = 0; i < 6; i++){
        S_cov[i][i] = S_cov[i][i] + R[i];
    }

    int k = 6;
    double d;
    d = determinant(S_cov, k);

    cofactor(S_cov, k, S_cov_inv);





    //* Kalman Gain matrix K = P*H_trans*S_cov_inv */

    multiply_rect3(P_local, H_trans, K_Temp);
    multiply_rect4(K_Temp, S_cov_inv, K_Gain);

    //* Updating State estimate */

    multiply_rect5(K_Gain, y_res, Del_x);

    for(i = 0; i<7; i++){
        X_state[i] = X_state[i] + Del_x[i];
    }

    //* Normalizing Quaternion -- Updated State*/

    qnorm = sqrt(X_state[0] * X_state[0] + X_state[1] * X_state[1] + X_state[2] * X_state [2] + X_state[3] * X_state[3]);
    q0 = X_state[0]/qnorm;
    q1 = X_state[1]/qnorm;
    q2 = X_state[2]/qnorm;
    q3 = X_state[3]/qnorm;

    //* State Information */

    X_state[0] = q0;
    X_state[1] = q1;
    X_state[2] = q2;
    X_state[3] = q3;

    sensorData.q[0] = q0;
    sensorData.q[1] = q1;
    sensorData.q[2] = q2;
    sensorData.q[3] = q3;

    ///* Updating Estimate Covariance P_new = ( I − K*H) *P  */

    multiply_rect6(K_Gain , H, P_temp);

    for(i=0; i<7; i++){
        for(j=0; j<7; j++){
            P_temp[i][j] = -P_temp[i][j];
        }
    }

    for(i = 0; i++; i<7){
        P_temp[i][i] = 1 + P_temp[i][i];
    }

    multiply_square(P_temp, P_local, P_new);


    /* Updating value  */

    for(i=0; i<7; i++){
        for(j=0; j<7; j++){
            P_extern[i][j] = P_new[i][j];
        }
    }

}


void multiply_square(double A[7][7], double B[7][7], double C[7][7])
{
    int i, j, k;

    for(i = 0 ; i<7; i++)
    {
        for(j = 0; j<7; j++)
        {
            C[i][j] = 0;

            for(k = 0; k<7; k++)
            {
                C[i][j] += A[i][k] *B[k][j];
            }
        }
    }
}

void multiply_rect1(double A[6][7], double B[7][7], double C[6][7])
{
    int i, j, k;

    for(i = 0 ; i<6; i++)
    {
        for(j = 0; j<7; j++)
        {
            C[i][j] = 0;

            for(k = 0; k<7; k++)
            {
                C[i][j] += A[i][k] *B[k][j];
            }
        }
    }
}


void multiply_rect2(double A[6][7], double B[7][6], double C[6][6])
{
    int i, j, k;

    for(i = 0 ; i<6; i++)
    {
        for(j = 0; j<6; j++)
        {
            C[i][j] = 0;

            for(k = 0; k<7; k++)
            {
                C[i][j] += A[i][k] *B[k][j];
            }
        }
    }
}


void multiply_rect3(double A[7][7], double B[7][6], double C[7][6])
{
    int i, j, k;

    for(i = 0 ; i<7; i++)
    {
        for(j = 0; j<6; j++)
        {
            C[i][j] = 0;

            for(k = 0; k<7; k++)
            {
                C[i][j] += A[i][k] *B[k][j];
            }
        }
    }
}


void multiply_rect4(double A[7][6], double B[6][6], double C[7][6])
{
    int i, j, k;

    for(i = 0 ; i<7; i++)
    {
        for(j = 0; j<6; j++)
        {
            C[i][j] = 0;

            for(k = 0; k<6; k++)
            {
                C[i][j] += A[i][k] *B[k][j];
            }
        }
    }
}


void multiply_rect5(double A[7][6], double B[6], double C[7])
{
    int i, j;

    for(i = 0 ; i<7; i++)
    {
        C[i] = 0;

        for(j = 0; j<6; j++)
        {
            C[i]+= A[i][j]*B[j];
        }
    }
}


void multiply_rect6(double A[7][6], double B[6][7], double C[7][7])
{
    int i, j, k;

    for(i = 0 ; i<7; i++)
    {
        for(j = 0; j<7; j++)
        {
            C[i][j] = 0;

            for(k = 0; k<6; k++)
            {
                C[i][j] += A[i][k] *B[k][j];
            }
        }
    }
}
    /*For calculating Determinant of the Matrix */

double determinant(double a[6][6], int k)
{

     double s = 1, det = 0, b[6][6];
     int i, j, m, n, c;

     if (k == 1)
       {
         return (a[0][0]);
       }
     else
       {
         det = 0;
         for (c = 0; c < k; c++)
   	{
   	  m = 0;
   	  n = 0;
   	  for (i = 0; i < k; i++)
   	    {
   	      for (j = 0; j < k; j++)
   		{
   		  b[i][j] = 0;

   		  if (i != 0 && j != c)
   		    {
   		      b[m][n] = a[i][j];

   		      if (n < (k - 2))
   			n++;
   		      else
   		      {
   			  n = 0;
   			  m++;
   		      }
   		    }
   		}
   	    }
   	  det = det + s * (a[0][c] * determinant (b, k - 1));
   	  s = -1 * s;
   	}
       }
     return (det);
   }


void cofactor(double num[6][6], int f, double S_cov_inv[6][6])
   {

     double b[6][6], fac[6][6];

     int p, q, m, n, i, j;

     for (q = 0; q < f; q++)

       {

         for (p = 0; p < f; p++)

   	{

   	  m = 0;

   	  n = 0;

   	  for (i = 0; i < f; i++)

   	    {

   	      for (j = 0; j < f; j++)

   		{

   		  if (i != q && j != p)

   		    {

   		      b[m][n] = num[i][j];

   		      if (n < (f - 2))

   			n++;

   		      else

   			{

   			  n = 0;

   			  m++;

   			}

   		    }

   		}

   	    }

   	  fac[q][p] = pow (-1, q + p) * determinant (b, f - 1);

   	}

       }

     transpose (num, fac, f, S_cov_inv);

   }

       /*Finding transpose of matrix */
void transpose(double num[6][6], double fac[6][6], int r, double S_cov_inv[6][6])
   {

     int i, j;

     double b[6][6], d;



     for (i = 0; i < r; i++)

       {

         for (j = 0; j < r; j++)

   	{

   	  b[i][j] = fac[j][i];

   	}

       }

     d = determinant (num, r);

     for (i = 0; i < r; i++)

       {

         for (j = 0; j < r; j++)

   	{

   	  S_cov_inv[i][j] = b[i][j] / d;

   	}
       }
   }



void EncoderInit()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;

  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* TIM2 channel 4 pin (PA3) configuration for Encoder A (Yellow)*/
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect TIM pins to AF2 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

  /* Configure (PA4) pin as input floating for Encoder B (White)*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* -----------------------------------------------------------------------
     TIM2 Configuration:

     In this example TIM2 input clock (TIM2CLK) is set to 2 * APB1 clock (PCLK1):
     	 TIM2CLK = SystemCoreClock / 2 = 84000000 Hz

     To get TIM2 counter clock at X Hz, the prescaler is computed as follows:
     	 Prescaler = (TIM3CLK / TIM3 counter clock) - 1
     	 Prescaler = ((SystemCoreClock /2) / X Hz) - 1

     Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
    ----------------------------------------------------------------------- */
  //TIM_PrescalerConfig(TIM2, (uint16_t) (((SystemCoreClock/2) / X) - 1), TIM_PSCReloadMode_Immediate);

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM2 configuration: Input Capture mode ---------------------
     The external signal is connected to TIM2 CH4 pin (PA3)
     The Rising edge is used as active edge,
     The TIM2 CCR4 is used to compute the frequency value
  ------------------------------------------------------------ */
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;		/*!< Capture performed once every 8 events. */
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  /* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
}

extern "C" {
/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM2, TIM_IT_CC4) == SET)
  {
    /* Clear TIM2 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
    CaptureTime = NOW();
    if(CaptureNumber == 0)
    {
      /* Get the Input Capture value */
      IC4ReadValue1 = TIM_GetCapture4(TIM2);
      CaptureNumber = 1;
    }
    else if(CaptureNumber == 1)
    {
      EncoderB = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4);
      /* Get the Input Capture value */
      IC4ReadValue2 = TIM_GetCapture4(TIM2);

      /* Capture computation */
      if (IC4ReadValue2 > IC4ReadValue1)
      {
        Capture = (IC4ReadValue2 - IC4ReadValue1);
      }
      else if (IC4ReadValue2 < IC4ReadValue1)
      {
        Capture = ((0xFFFFFFFF - IC4ReadValue1) + IC4ReadValue2);
      }
      /* Frequency computation */
      TIM2Freq = (uint32_t) ((SystemCoreClock/2)) * 8 / Capture;
      CaptureNumber = 0;
    }
  }
}
}

void MotorSpeedUpdate()
{
	double SensorTime = ((NOW()-CaptureTime)/(double)MILLISECONDS);
	if (SensorTime>250) //minimum measured speed is 2 RPS(120 RPM). This can give us 250ms of minimum interval between interrupts (2 interrupts every one revolution).
	{
		TIM2Freq=0;
	}

	if (EncoderB)
	{
		sensorData.motorSpeed  = -1*((float)TIM2Freq / 16) * 60;  //CCW
	}
	else {sensorData.motorSpeed  = ((float)TIM2Freq / 16) * 60;}  //CW
}


class Sensors: public Thread {

	uint64_t periode;

public:

	Sensors(const char* name, uint64_t periode) : Thread(name) {
		this->periode = periode;
	}

	void init() {
	}

	void run() {

		EncoderInit();

		 if (imu.beginSPI() == false) // note, we need to sent this our CS pins (defined above)
		  {
			 PRINTF("Failed to communicate with LSM9DS1.\r\n");
		  }
		 /*printGyro();  // Print "G: gx, gy, gz"
		 printAccel(); // Print "A: ax, ay, az"
		 printMag();   // Print "M: mx, my, mz"
		 GyroCal(gbias);
		 AccelCal(abias);
		 MagCal(magMax, magMin, 10);
*/
		TIME_LOOP(0, periode){
			PRINTF("I am working");
			printGyro();           // Read raw gyro data
					sensorData.gx =round(imu.calcGyro(imu.gx - gbias[0]));   // Convert to degrees per seconds, remove gyro biases
					sensorData.gy =round(imu.calcGyro(imu.gy - gbias[1]));
					sensorData.gz =round(imu.calcGyro(imu.gz - gbias[2]));
		PRINTF("I am working");
					printAccel(); // Print "A: ax, ay, az"         // Read raw accelerometer data
					sensorData.ax =imu.calcAccel(imu.ax -  abias[0]);   // Convert to g's, remove accelerometer biases
					sensorData.ay =imu.calcAccel(imu.ay -  abias[1]);
					sensorData.az =imu.calcAccel(imu.az -  abias[2]);

					printMag();         // Read raw magnetometer data
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
					PRINTF("I am working33 ");

					//The LSM9DS1's magnetometer x and y axes are opposite to the accelerometer, so my and mx are substituted for each other.
					ExtendedKalmanFilter(sensorData.ax, sensorData.ay, sensorData.az, sensorData.gx, sensorData.gy, sensorData.gz, -magData[1], -magData[0], magData[2]);  // Pass gyro rate as rad/s
					Quaternion2Euler();
					PRINTF("x = %f  y = %f  z = %f", sensorData.roll,sensorData.pitch, sensorData.yaw );




			  // Print the heading and orientation for fun!
			  // Call print attitude. The LSM9DS1's magnetometer x and y
			  // axes are opposite to the accelerometer, so my and mx are
			  // substituted for each other.
			printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);

			/*MotorSpeedUpdate();

			SensorDataTopic.publish(sensorData);*/

			suspendCallerUntil(NOW()+20*MILLISECONDS);
		}
	}
};
Sensors Sensors("Sensors", SensorsPeriod * MILLISECONDS);


/***********************************************************************/
