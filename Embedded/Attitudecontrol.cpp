/*#
 * Template.cpp
 *
 * Created on: 25.06.2014
 * Author: Atheel Redah
 *
 */

#include "rodos.h"
#include <stdio.h>
#include "math.h"
#include "topics.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#define HBRIDGE_IN11      GPIO_065					// PE1 = IN1-1 = 36
#define HBRIDGE_IN12      GPIO_038					// PC6 = IN1-2 = 37
#define HBRIDGE_IN13      GPIO_008					// PA8 = IN1-3 = 43

#define HBRIDGE_IN21      GPIO_022					// PB6 = IN2-1 = 47
#define HBRIDGE_IN22      GPIO_005					// PA5 = IN2-2 = 01
#define HBRIDGE_IN23      GPIO_004					// PA4 = IN2-3 = 04

// DC Motor on the X Axis

HAL_PWM  Motor_PWM_X(PWM_IDX13);   // PB9 = D1
HAL_GPIO Motor_IN_X1(HBRIDGE_IN13);
HAL_GPIO Motor_IN_X2(HBRIDGE_IN23);


// DC Motor on the Y Axis

HAL_PWM  Motor_PWM_Y(PWM_IDX14);   // PB9 = D1
HAL_GPIO Motor_IN_Y1(HBRIDGE_IN12);
HAL_GPIO Motor_IN_Y2(HBRIDGE_IN22);


// DC Motor on the -Z Axis

HAL_PWM  Motor_PWM_Z(PWM_IDX15);   // PB9 = D1
HAL_GPIO Motor_IN_Z1(HBRIDGE_IN11);
HAL_GPIO Motor_IN_Z2(HBRIDGE_IN21);

// Encoders Variables

// Encoder:1, Axe: X
__IO uint32_t IC3ReadValue_X_1 = 0, IC3ReadValue_X_2 = 0, Capture_X = 0;
__IO uint8_t CaptureNumber_X = 0;
__IO uint32_t X_TIM5Freq = 0;
__IO uint8_t EncoderBX;
__IO double CaptureTime_X;

// Encoder:2, Axe: Y
__IO uint32_t IC4ReadValue_Y_1 = 0, IC4ReadValue_Y_2 = 0, Capture_Y = 0;
__IO uint8_t CaptureNumber_Y = 0;
__IO uint32_t Y_TIM2Freq = 0;
__IO uint8_t EncoderBY;
__IO double CaptureTime_Y;

// Encoder:3, Axe: Z
__IO uint32_t IC1ReadValue_Z_1 = 0, IC1ReadValue_Z_2 = 0, Capture_Z = 0;
__IO uint8_t CaptureNumber_Z = 0;
__IO uint32_t Z_TIM5Freq = 0;
__IO uint8_t EncoderBZ;
__IO double CaptureTime_Z;



// --------------#--------------#--------------#-------------#-------------#--------------#---------------#-------------


CommBuffer<Telecommandmsg> TelecommandDataBuffer3;
Subscriber TelecommandDataSubscriber3(TelecommandDataTopic, TelecommandDataBuffer3);

CommBuffer<ImuDatamsg> IMUDataBuffer3;
Subscriber IMUDataSubscriber3(ImuTopic, IMUDataBuffer3);

CommBuffer<APmsg> APDataBuffer3;
Subscriber APDataSubscriber3(AttitudeplanningTopic, APDataBuffer3);

Telecommandmsg TelecommandDataReceiver2; //Input
ImuDatamsg ImuDataReceiver2; //Input
AC2Telemetrie ACmsg; //Output
APmsg nextMove;

int thread_period = 351; // milliseconds

// X DCMotor PID controller

sPID_Data PID_MotorSpeed = {1, 0.5, 0,                               // float Kp, Ki, Kd;
		0, 0, 0, 							     // float P, I, D;
		0, 0, 0, 0, 0, 0, 1,                      // float e, e_1, Upid, Upid_1, Usat, Usat_1, Kr;
		8000, -8000, (float)thread_period/1000,     // float Umax, Umin, T;
		1 // bool AntiWindup;
};

// Y DCMotor PID controller
sPID_Data PID_MotorSpeed1 = {1, 0.5, 0,                               // float Kp, Ki, Kd;
		0, 0, 0, 							     // float P, I, D;
		0, 0, 0, 0, 0, 0, 1,                      // float e, e_1, Upid, Upid_1, Usat, Usat_1, Kr;
		8000, -8000, (float)thread_period/1000,     // float Umax, Umin, T;
		1 // bool AntiWindup;
};

// Z DCMotor PID controller
sPID_Data PID_MotorSpeed2 = {1, 0.5, 0,                               // float Kp, Ki, Kd;
		0, 0, 0, 							     // float P, I, D;
		0, 0, 0, 0, 0, 0, 1,                      // float e, e_1, Upid, Upid_1, Usat, Usat_1, Kr;
		8000, -8000, (float)thread_period/1000,     // float Umax, Umin, T;
		1 // bool AntiWindup
};

float MotorSpeedError[3];

// --------------#--------------#--------------#-------------#-------------#--------------#---------------#-------------

class Attitudecontrol: public Thread {

public:

	Attitudecontrol(const char* name) : Thread(name) {

	}

	void Encoder_X_Init(){
		GPIO_InitTypeDef GPIO_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		TIM_ICInitTypeDef  TIM_ICInitStructure;

		/* TIM5 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

		/* GPIOA clock enable */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		/* TIM2 channel 3 pin (PA2) configuration for Encoder A (Yellow)*/
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Connect TIM pins to AF2 */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);

		/* Configure (PA1) pin as input floating for Encoder B (White)*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Enable the TIM5 global Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/* TIM2 configuration: Input Capture mode ---------------------
	     The external signal is connected to TIM5 CH3 pin (PA2)
	     The Rising edge is used as active edge,
	     The TIM5 CCR3 is used to compute the frequency value
	  ------------------------------------------------------------ */
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;		/*!< Capture performed once every 8 events. */
		TIM_ICInitStructure.TIM_ICFilter = 0x0;

		TIM_ICInit(TIM5, &TIM_ICInitStructure);

		/* TIM enable counter */
		TIM_Cmd(TIM5, ENABLE);

		/* Enable the CC3 Interrupt Request */
		TIM_ITConfig(TIM5, TIM_IT_CC3, ENABLE);
	}
	void Encoder_Y_Init(){
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
	void Encoder_Z_Init(){
		GPIO_InitTypeDef GPIO_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		TIM_ICInitTypeDef  TIM_ICInitStructure;

		/* TIM5 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); // RCC_APB2Periph_TIM8

		/* GPIOA clock enable */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		/* TIM5 channel 1 pin (PA0) configuration for Encoder A (Yellow)*/
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Connect TIM pins to AF2 */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);

		/* Configure (PC7) pin as input floating for Encoder B (White)*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		/* Enable the TIM5 global Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/* TIM5 configuration: Input Capture mode ---------------------
	     The external signal is connected to TIM5 CH1 pin (PC7)
	     The Rising edge is used as active edge,
	     The TIM5 CCR1 is used to compute the frequency value
	  ------------------------------------------------------------ */
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;		/*!< Capture performed once every 8 events. */
		TIM_ICInitStructure.TIM_ICFilter = 0x0;

		TIM_ICInit(TIM5, &TIM_ICInitStructure);

		/* TIM enable counter */
		TIM_Cmd(TIM5, ENABLE);

		/* Enable the CC2 Interrupt Request */
		TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);
	}

	void X_TIM5_IRQHandler(void){

		if(TIM_GetITStatus(TIM5, TIM_IT_CC3) == SET)
		{
			/* Clear TIM2 Capture compare interrupt pending bit */
			TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);
			CaptureTime_X = NOW();
			if(CaptureNumber_X == 0)
			{
				/* Get the Input Capture value */
				IC3ReadValue_X_1 = TIM_GetCapture3(TIM5);
				CaptureNumber_X = 1;
			}
			else if(CaptureNumber_X == 1)
			{
				EncoderBX = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
				/* Get the Input Capture value */
				IC3ReadValue_X_2 = TIM_GetCapture3(TIM5);

				/* Capture computation */
				if (IC3ReadValue_X_2 > IC3ReadValue_X_1)
				{
					Capture_X = (IC3ReadValue_X_2 - IC3ReadValue_X_1);
				}
				else if (IC3ReadValue_X_2 < IC3ReadValue_X_1)
				{
					Capture_X = ((0xFFFFFFFF - IC3ReadValue_X_1) + IC3ReadValue_X_2);
				}
				/* Frequency computation */
				X_TIM5Freq = (uint32_t) ((SystemCoreClock/2)) * 8 / Capture_X;
				CaptureNumber_X = 0;
			}
		}
	}
	void Y_TIM2_IRQHandler(void){

		if(TIM_GetITStatus(TIM2, TIM_IT_CC4) == SET)
		{
			/* Clear TIM2 Capture compare interrupt pending bit */
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
			CaptureTime_Y = NOW();
			if(CaptureNumber_Y == 0)
			{
				/* Get the Input Capture value */
				IC4ReadValue_Y_1 = TIM_GetCapture4(TIM2);
				CaptureNumber_Y = 1;
			}
			else if(CaptureNumber_Y == 1)
			{
				EncoderBY = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4);
				/* Get the Input Capture value */
				IC4ReadValue_Y_2 = TIM_GetCapture4(TIM2);

				/* Capture computation */
				if (IC4ReadValue_Y_2 > IC4ReadValue_Y_1)
				{
					Capture_Y = (IC4ReadValue_Y_2 - IC4ReadValue_Y_1);
				}
				else if (IC4ReadValue_Y_2 < IC4ReadValue_Y_1)
				{
					Capture_Y = ((0xFFFFFFFF - IC4ReadValue_Y_1) + IC4ReadValue_Y_2);
				}
				/* Frequency computation */
				Y_TIM2Freq = (uint32_t) ((SystemCoreClock/2)) * 8 / Capture_Y;
				CaptureNumber_Y = 0;
			}
		}
	}
	void Z_TIM5_IRQHandler(void){

		if(TIM_GetITStatus(TIM5, TIM_IT_CC1) == SET)
		{
			/* Clear TIM2 Capture compare interrupt pending bit */
			TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
			CaptureTime_Z = NOW();
			if(CaptureNumber_Z == 0)
			{
				/* Get the Input Capture value */
				IC1ReadValue_Z_1 = TIM_GetCapture1(TIM5);
				CaptureNumber_Z = 1;
			}
			else if(CaptureNumber_Z == 1)
			{
				EncoderBZ = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7);
				/* Get the Input Capture value */
				IC1ReadValue_Z_2 = TIM_GetCapture1(TIM5);

				/* Capture computation */
				if (IC1ReadValue_Z_2 > IC1ReadValue_Z_1)
				{
					Capture_Z = (IC1ReadValue_Z_2 - IC1ReadValue_Z_1);
				}
				else if (IC1ReadValue_Z_2 < IC1ReadValue_Z_1)
				{
					Capture_Z = ((0xFFFFFFFF - IC1ReadValue_Z_1) + IC1ReadValue_Z_2);
				}
				/* Frequency computation */
				Z_TIM5Freq = (uint32_t) ((SystemCoreClock/2)) * 8 / Capture_Z;
				CaptureNumber_Z = 0;
			}
		}
	}

	void Motor_X_SpeedUpdate(int32_t* motorspeed)
	{
		double SensorTime = ((NOW()-CaptureTime_X)/(double)MILLISECONDS);
		if (SensorTime>250) //minimum measured speed is 2 RPS(120 RPM). This can give us 250ms of minimum interval between interrupts (2 interrupts every one revolution).
		{
			X_TIM5Freq=0;
		}

		if (EncoderBX)
		{
			*motorspeed  = -1*((float)X_TIM5Freq / 16) * 60;  //CCW
		}
		else {
			*motorspeed  = ((float)X_TIM5Freq / 16) * 60;}  //CW
	}
	void Motor_Y_SpeedUpdate(int32_t* motorspeed)
	{
		double SensorTime = ((NOW()-CaptureTime_Y)/(double)MILLISECONDS);
		if (SensorTime>250) //minimum measured speed is 2 RPS(120 RPM). This can give us 250ms of minimum interval between interrupts (2 interrupts every one revolution).
		{
			Y_TIM2Freq=0;
		}

		if (EncoderBY)
		{
			*motorspeed  = -1*((float)Y_TIM2Freq / 16) * 60;  //CCW
		}
		else {
			*motorspeed  = ((float)Y_TIM2Freq / 16) * 60;}  //CW
	}
	void Motor_Z_SpeedUpdate(int32_t* motorspeed)
	{
		double SensorTime = ((NOW()-CaptureTime_Z)/(double)MILLISECONDS);
		if (SensorTime>250) //minimum measured speed is 2 RPS(120 RPM). This can give us 250ms of minimum interval between interrupts (2 interrupts every one revolution).
		{
			Z_TIM5Freq=0;
		}

		if (EncoderBZ)
		{
			*motorspeed  = -1*((float)Z_TIM5Freq / 16) * 60;  //CCW
		}
		else {
			*motorspeed  = ((float)Z_TIM5Freq / 16) * 60;}  //CW
	}

	void PID(sPID_Data* PID, float error)
	{
		PID->e = (float) error;

		PID->P = PID->Kp*PID->e;

		if (PID->AntiWindup){
			PID->I+= (PID->Ki*PID->e + PID->Kr * (PID->Usat_1 - PID->Upid_1)) * PID->T;
		}else {PID->I+= PID->Ki * PID->e * PID->T;
		}

		PID->D = PID->Kd * (PID->e - PID->e_1) / PID->T;

		PID->Upid = PID->P + PID->I + PID->D;

		if (PID->Upid >= PID->Umax){
			PID->Usat = PID->Umax;
		}else if (PID->Upid <= PID->Umin){
			PID->Usat = PID->Umin;
		}else{
			PID->Usat = PID->Upid;
		}

		PID->Upid_1 = PID->Upid;
		PID->Usat_1 = PID->Usat;
		PID->e_1 = PID->e;
	}

	void MotorSet(float Value,int index){

		if(index == 0){
			if (Value>=0){
				Motor_IN_Z1.setPins(1);
				Motor_IN_Z2.setPins(0);
			}else{
				Value=-1*Value;
				Motor_IN_Z1.setPins(0);
				Motor_IN_Z2.setPins(1);
			}
			Motor_PWM_Z.write((unsigned int) Value);
		}else if(index == 1){
			if (Value>=0){
				Motor_IN_Y1.setPins(1);
				Motor_IN_Y2.setPins(0);
			}else{
				Value=-1*Value;
				Motor_IN_Y1.setPins(0);
				Motor_IN_Y2.setPins(1);
			}
			Motor_PWM_Y.write((unsigned int) Value);
		}else if(index == 2){
			if (Value>=0){
				Motor_IN_X1.setPins(1);
				Motor_IN_X2.setPins(0);
			}else{
				Value=-1*Value;
				Motor_IN_X1.setPins(0);
				Motor_IN_X2.setPins(1);
			}
			Motor_PWM_X.write((unsigned int) Value);
		}
	}

	void StandbyMode()
	{
		Motor_PWM_X.write(0);
		Motor_IN_X1.setPins(0);
		Motor_IN_X2.setPins(0);

		Motor_PWM_Y.write(0);
		Motor_IN_Y1.setPins(0);
		Motor_IN_Y2.setPins(0);

		Motor_PWM_Z.write(0);
		Motor_IN_Z1.setPins(0);
		Motor_IN_Z2.setPins(0);
	}

	void SteeringMode(float *ref_DCspeed,float *sens_DCspeed)
	{
		*(MotorSpeedError + 0) = *(ref_DCspeed + 0) - *(sens_DCspeed + 0);
		*(MotorSpeedError + 1) = *(ref_DCspeed + 1) - *(sens_DCspeed + 1);
		*(MotorSpeedError + 2) = *(ref_DCspeed + 2) - *(sens_DCspeed + 2);

		PID(&PID_MotorSpeed, *(MotorSpeedError + 0));
		PID(&PID_MotorSpeed1, *(MotorSpeedError + 1));
		PID(&PID_MotorSpeed2, *(MotorSpeedError + 2));

		// For the Motor X
		if (*(ref_DCspeed + 0) == 0 && *(sens_DCspeed + 0) == 0){
			Motor_PWM_X.write(0);
			Motor_IN_X1.setPins(0);
			Motor_IN_X2.setPins(0);
		}else{
			MotorSet(PID_MotorSpeed.Usat,0);
		}

		// For the Motor Y
		if (*(ref_DCspeed + 1) == 0 && *(sens_DCspeed + 1) == 0){
			Motor_PWM_Y.write(0);
			Motor_IN_Y1.setPins(0);
			Motor_IN_Y2.setPins(0);
		}else{
			MotorSet(PID_MotorSpeed1.Usat,1);
		}

		// For the Motor Z
		if (*(ref_DCspeed + 2) == 0 && *(sens_DCspeed + 2) == 0){
			Motor_PWM_Z.write(0);
			Motor_IN_Z1.setPins(0);
			Motor_IN_Z2.setPins(0);
		}else{
			MotorSet(PID_MotorSpeed2.Usat,2);
		}
	}

	void init(void) {
	}

	void run() {

		float deltaTime;
		float timeNow;

		Encoder_X_Init();
		Encoder_Y_Init();
		Encoder_Z_Init();

		Motor_PWM_X.init(1000,1000);
		Motor_IN_X1.init(true, 1, 0);
		Motor_IN_X2.init(true, 1, 0);

		Motor_PWM_Y.init(1000,1000);
		Motor_IN_Y1.init(true, 1, 0);
		Motor_IN_Y2.init(true, 1, 0);

		Motor_PWM_Z.init(1000,1000);
		Motor_IN_Z1.init(true, 1, 0);
		Motor_IN_Z2.init(true, 1, 0);

		TelecommandDataReceiver2.SystemMode = 15;

		//TIME_LOOP(0,this->thread_period){
		while(1){

			TelecommandDataBuffer3.getOnlyIfNewData(TelecommandDataReceiver2); // Check the Telecommand buffer for new Telecommands
			IMUDataBuffer3.getOnlyIfNewData(ImuDataReceiver2); // call imu for sat orientation
			APDataBuffer3.getOnlyIfNewData(nextMove); // call Attitude planning for next sat move


			// Read the Encoders and store the velocity in RPM for each axes
			Motor_X_SpeedUpdate(&ACmsg.DCSpeed[0]);
			Motor_Y_SpeedUpdate(&ACmsg.DCSpeed[1]);
			Motor_Z_SpeedUpdate(&ACmsg.DCSpeed[2]);

			// compute the deltatime
			timeNow = NOW();
			deltaTime = ((timeNow - deltaTime) / (double)SECONDS);

			if(TelecommandDataReceiver2.SystemMode == 1){
				//  Call for the Motor Controller 1
				motorcontrol_Run(nextMove.euler, ImuDataReceiver2.acceleration, deltaTime,TelecommandDataReceiver2.Is,
						TelecommandDataReceiver2.Irw, TelecommandDataReceiver2.DCSpeed);
			}

			PRINTF("checking to go inside the controller with the value: %d \r\n",TelecommandDataReceiver2.SystemMode);

			if(TelecommandDataReceiver2.SystemMode == 2 || TelecommandDataReceiver2.SystemMode == 1){

				// compute the velocity error for the controller 2
				for(int i = 0;i<3;i++){
					MotorSpeedError[i] = TelecommandDataReceiver2.DCSpeed[i] - ACmsg.DCSpeed[i];
				}

				// Call for the Motor Controller 2
				PID(&PID_MotorSpeed, MotorSpeedError[0]);
				PID(&PID_MotorSpeed1, MotorSpeedError[1]);
				PID(&PID_MotorSpeed2, MotorSpeedError[2]);

				// Send the calculated RPM to the Motorset method
				if (TelecommandDataReceiver2.DCSpeed[0] == 0)
				{
					Motor_PWM_X.write(0);
					Motor_IN_X1.setPins(0);
					Motor_IN_X2.setPins(0);
				}
				else{MotorSet(PID_MotorSpeed.Usat, 0);
				PRINTF("I'm in the Motor %d with the value: %d \r\n",0,TelecommandDataReceiver2.DCSpeed[0]);
				}

				if ( TelecommandDataReceiver2.DCSpeed[1] == 0)
				{
					Motor_PWM_Y.write(0);
					Motor_IN_Y1.setPins(0);
					Motor_IN_Y2.setPins(0);
				}
				else{ MotorSet(PID_MotorSpeed1.Usat, 1);
				PRINTF("I'm in the Motor %d with the value: %d \r\n",1,TelecommandDataReceiver2.DCSpeed[1]);
				}

				if (TelecommandDataReceiver2.DCSpeed[2] == 0)
				{
					Motor_PWM_Z.write(0);
					Motor_IN_Z1.setPins(0);
					Motor_IN_Z2.setPins(0);
				}
				else{MotorSet(PID_MotorSpeed2.Usat, 2);
				PRINTF("I'm in the Motor %d with the value: %d \r\n",2,TelecommandDataReceiver2.DCSpeed[2]);
				}
			}

			thread_period = TelecommandDataReceiver2.ThreadPeriod[3] == -1? thread_period:TelecommandDataReceiver2.ThreadPeriod[3];
			PRINTF("The periode for the controller is : %d \r\n",thread_period);

			deltaTime = timeNow;

			AC2TelemetrieTopic.publish(ACmsg);
			suspendCallerUntil(NOW() + thread_period*MILLISECONDS);
		}
	}
};

Attitudecontrol Attitudecontrol("Attitudecontrol");

/***********************************************************************/
