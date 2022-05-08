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
#include "topics.h"

static Application module01("Template", 2001);

uint32_t ModesPeriod = 20; // Modes period in ms

#define LED_BLUE GPIO_060    //PD12
#define LED_RED GPIO_061  //PD13
#define LED_ORANGE GPIO_062  //PD14
#define LED_GREEN GPIO_063   //PD15


struct sPID_Data
{
	 float Kp, Ki, Kd;

	 float P, I, D;

	 float e, e_1, Upid, Upid_1, Usat, Usat_1, Kr;

	 float Umax, Umin, T;

	 bool AntiWindup;
};

sPID_Data PID_MotorSpeed = {1, 0.5, 0,                               // float Kp, Ki, Kd;
					       0, 0, 0, 							     // float P, I, D;
					       0, 0, 0, 0, 0, 0, 1,                      // float e, e_1, Upid, Upid_1, Usat, Usat_1, Kr;
					       4000, -4000, (float)ModesPeriod/1000,     // float Umax, Umin, T;
                           1};                                       // bool AntiWindup;


float MotorSpeedError;

//#define HBRIDGE_PWM      GPIO_023 	            // PB7 = TIM4_CH2 = PWM_IDX13 = D3 = 46
//#define HBRIDGE_PWM      GPIO_024 	            // PB8 = TIM4_CH3 = D2 = 45
//#define HBRIDGE_PWM      GPIO_025 	            // PB9 = TIM4_CH4 = D1 = 44

#define HBRIDGE_IN11      GPIO_065					// PE1 = IN1-1 = 36
#define HBRIDGE_IN12      GPIO_038					// PC6 = IN1-2 = 37
#define HBRIDGE_IN13      GPIO_008					// PA8 = IN1-3 = 43

#define HBRIDGE_IN21      GPIO_022					// PB6 = IN2-1 = 47
#define HBRIDGE_IN22      GPIO_005					// PA5 = IN2-2 = 01
#define HBRIDGE_IN23      GPIO_004					// PA4 = IN2-3 = 04

 //#define HBRIDGE_FB       GPIO_000					// PA0 = FB0 = 08
//#define HBRIDGE_FB       GPIO_001					// PA1 = FB1 = 07
//#define HBRIDGE_FB       GPIO_002					// PA2 = FB2 = 06


HAL_PWM  Motor_PWM(PWM_IDX15);   // PB9 = D1
HAL_GPIO Motor_IN1(HBRIDGE_IN11);
HAL_GPIO Motor_IN2(HBRIDGE_IN21);

HAL_PWM  Motor_PWM1(PWM_IDX14);   // PB9 = D1
HAL_GPIO Motor_IN12(HBRIDGE_IN12);
HAL_GPIO Motor_IN22(HBRIDGE_IN22);

HAL_PWM  Motor_PWM2(PWM_IDX13);   // PB9 = D1
HAL_GPIO Motor_IN13(HBRIDGE_IN13);
HAL_GPIO Motor_IN23(HBRIDGE_IN23);


enum Operation_Mode
{
	Standby_Mode     = 1,
	Steering_Mode    = 2,
};

enum Motor
{
	Motor1    = 1,
	Motor2    = 2,
	Motor3    = 3,
};

uint8_t SystemMode = Standby_Mode;
uint8_t Motor;



CommBuffer<sSensorData> ModesSensorDataBuffer; //ModesSensorDataBuffer1, ModesSensorDataBuffer2;
Subscriber ModesSensorSubscriber(SensorDataTopic, ModesSensorDataBuffer);
//Subscriber ModesSensorSubscriber(SensorDataTopic, ModesSensorDataBuffer1);
//Subscriber ModesSensorSubscriber(SensorDataTopic, ModesSensorDataBuffer2);

CommBuffer<sTelecommandData> ModesTelecommandDataBuffer; //ModesTelecommandDataBuffer1, ModesTelecommandDataBuffer2;
Subscriber ModesTelecommandDataSubscriber(TelecommandDataTopic, ModesTelecommandDataBuffer);
//subscriber ModesTelecommandDataSubscriber(TelecommandDataTopic, ModesTelecommandDataBuffer1);
//Subscriber ModesTelecommandDataSubscriber(TelecommandDataTopic, ModesTelecommandDataBuffer2);

sSensorData SensorDataReceiver; //SensorDataReceiver1, SensorDataReceiver2;

sTelecommandData TelecommandDataReceiver; //TelecommandDataReceiver1, TelecommandDataReceiver2;


void PID(sPID_Data* PID, float error)
{
	PID->e = (float) error;

	PID->P = PID->Kp*PID->e;

	if (PID->AntiWindup)
	{
		PID->I+= (PID->Ki*PID->e + PID->Kr * (PID->Usat_1 - PID->Upid_1)) * PID->T;
	}

	else {PID->I+= PID->Ki * PID->e * PID->T;}

	PID->D = PID->Kd * (PID->e - PID->e_1) / PID->T;

	PID->Upid = PID->P + PID->I + PID->D;

	if (PID->Upid >= PID->Umax)
	{
		PID->Usat = PID->Umax;
	}

	else if (PID->Upid <= PID->Umin)
	{
		PID->Usat = PID->Umin;
	}

	else
	{
		PID->Usat = PID->Upid;
	}

	PID->Upid_1 = PID->Upid;
	PID->Usat_1 = PID->Usat;
	PID->e_1 = PID->e;
}

void MotorSet(float Value, int motor)
{
	switch (motor) {

	case 1:
	if (Value>=0)
	{
		Motor_IN1.setPins(1);
		Motor_IN2.setPins(0);
	}
	else
	{
		Value=-1*Value;
		Motor_IN1.setPins(0);
		Motor_IN2.setPins(1);
	}
	Motor_PWM.write((unsigned int) Value);
	break;

	case 2:
		if (Value>=0)
			{
				Motor_IN12.setPins(1);
				Motor_IN22.setPins(0);
			}
			else
			{
				Value=-1*Value;
				Motor_IN12.setPins(0);
				Motor_IN22.setPins(1);
			}
			Motor_PWM1.write((unsigned int) Value);
			break;

	case 3:
			if (Value>=0)
				{
					Motor_IN13.setPins(1);
					Motor_IN23.setPins(0);
				}
				else
				{
					Value=-1*Value;
					Motor_IN13.setPins(0);
					Motor_IN23.setPins(1);
				}
				Motor_PWM2.write((unsigned int) Value);
				break;



	}
}

void StandbyMode(int motor)
{
	switch(motor) {

	case 1:
	    Motor_PWM.write(0);
     	Motor_IN1.setPins(0);
    	Motor_IN2.setPins(0);
	    break;

	case 2:
		Motor_PWM1.write(0);
		Motor_IN12.setPins(0);
		Motor_IN22.setPins(0);
		break;

	case 3:
		Motor_PWM2.write(0);
		Motor_IN13.setPins(0);
		Motor_IN23.setPins(0);
		break;

	}
}

void SteeringMode(int motor)
{
	switch(motor) {

	case 1:
	    MotorSpeedError = TelecommandDataReceiver.MotorSpeed - SensorDataReceiver.motorSpeed;
	    PID(&PID_MotorSpeed, MotorSpeedError);
	    if (SensorDataReceiver.motorSpeed==0 && TelecommandDataReceiver.MotorSpeed==0)
	    {
        Motor_PWM.write(0);
	    Motor_IN1.setPins(0);
	    Motor_IN2.setPins(0);
	    }
	    else{MotorSet(PID_MotorSpeed.Usat, 1);}
	    break;

	case 2:
		MotorSpeedError = TelecommandDataReceiver.MotorSpeed - SensorDataReceiver.motorSpeed;
		PID(&PID_MotorSpeed, MotorSpeedError);
		if (SensorDataReceiver.motorSpeed==0 && TelecommandDataReceiver.MotorSpeed==0)
		{
	    Motor_PWM1.write(0);
		Motor_IN12.setPins(0);
		Motor_IN22.setPins(0);
		}
		else{MotorSet(PID_MotorSpeed.Usat, 2);}
		break;

	case 3:
		MotorSpeedError = TelecommandDataReceiver.MotorSpeed - SensorDataReceiver.motorSpeed;
		PID(&PID_MotorSpeed, MotorSpeedError);
		if (SensorDataReceiver.motorSpeed==0 && TelecommandDataReceiver.MotorSpeed==0)
		{
		Motor_PWM2.write(0);
		Motor_IN13.setPins(0);
		Motor_IN23.setPins(0);
		}
		else{MotorSet(PID_MotorSpeed.Usat, 3);}
		break;
	}
}

class Modes: public Thread {

	uint64_t periode;


public:

	Modes(const char* name, uint64_t periode) : Thread(name) {
			this->periode = periode;
	}

	void init() {

		Motor_PWM.init(1000,1000);
		Motor_IN1.init(true, 1, 0);
		Motor_IN2.init(true, 1, 0);

		Motor_PWM1.init(1000,1000);
		Motor_IN12.init(true, 1, 0);
	    Motor_IN22.init(true, 1, 0);

	    Motor_PWM2.init(1000,1000);
	    Motor_IN13.init(true, 1, 0);
	    Motor_IN23.init(true, 1, 0);

	}

	void run() {


		TIME_LOOP(0, periode){

			ModesSensorDataBuffer.getOnlyIfNewData(SensorDataReceiver);

			ModesTelecommandDataBuffer.getOnlyIfNewData(TelecommandDataReceiver);

			if (TelecommandDataReceiver.Motor > 0)
						{Motor = TelecommandDataReceiver.Motor;}


			switch (Motor) {

			case (Motor1):

			if (TelecommandDataReceiver.SystemMode > 0)
			{SystemMode = TelecommandDataReceiver.SystemMode;}

			switch (SystemMode)
				{
				case Standby_Mode:
					StandbyMode(1);
					//MotorSet(200);
					break;
				case Steering_Mode:
					SteeringMode(1);
					break;
				default:
					break;
				};
			break;

			case Motor2:
				if (TelecommandDataReceiver.SystemMode > 0)
							{SystemMode = TelecommandDataReceiver.SystemMode;}

							switch (SystemMode)
								{
								case Standby_Mode:
									StandbyMode(2);
									//MotorSet(200);
									break;
								case Steering_Mode:
									SteeringMode(2);
									break;
								default:
									break;
								}
							break;

			case Motor3:
				if (TelecommandDataReceiver.SystemMode > 0)
							{SystemMode = TelecommandDataReceiver.SystemMode;}

							switch (SystemMode)
								{
								case Standby_Mode:
									StandbyMode(3);
									//MotorSet(200);
									break;
								case Steering_Mode:
									SteeringMode(3);
									break;
								default:
									break;
								}
							break;



			//PRINTF("system Mode is %d \r\n",SystemMode);

	}
	}
	}
};
Modes Modes("Modes", ModesPeriod * MILLISECONDS);




/***********************************************************************/
