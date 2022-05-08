

/*****************************************************************
Telecommand.cpp

Original Created by: Atheel Redah @ University of Würzburg
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4 + Würzburg Uni Informatik 8 Discovery AddOn Board Version 2.0.
*****************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "Telecommand.h"

static Application module01("Atheel2", 2001);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t ReceiveState = 0;
uint8_t SignFlag = 0;
uint8_t	DotFlag = 0;
uint8_t DataIndex = 0;
char TelecommandID;
char TelecommandIDNr;
char ReceiveData[MaxLength];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

namespace RODOS {
extern HAL_UART uart_stdout;
}
#define TeleUART uart_stdout

sTelecommandData TelecommandData;

uint8_t Decode(uint8_t RxBuffer)
{
	uint8_t success=0;

	switch (ReceiveState){

	case 0:
		SignFlag=0;
		DotFlag=0;
		DataIndex=0;
		if (RxBuffer==TelecommandStart)
		{
			ReceiveState=1;
		}
		break;

	case 1:
		SignFlag=0;
		DotFlag=0;
		DataIndex=0;
		if (RxBuffer==TelecommandStart)
		{
			ReceiveState=1;
		}
		else if (RxBuffer==TelecommandStop)
		{
				ReceiveState=0;
		}
		else {
			TelecommandID = RxBuffer;
			ReceiveState = 2;
		}
		break;

	case 2:
		if (RxBuffer==TelecommandStart)
		{
			ReceiveState=1;
		}
		else if (RxBuffer>='0' && RxBuffer<='9')
		{
			TelecommandIDNr = RxBuffer;
			ReceiveState = 3;
		}
		else
		{
			ReceiveState=0;
		}
		break;

	case 3:
		if (RxBuffer=='+' || RxBuffer=='-')
		{
			if (SignFlag==0 && DataIndex==0)
				{
				SignFlag=1;
				ReceiveData[DataIndex]=RxBuffer;
				DataIndex++;
				ReceiveState = 3;
				}
			else {ReceiveState = 0;}
		}
		else if (RxBuffer=='.')
		{
			if (DotFlag==0)
				{
				DotFlag=1;
				ReceiveData[DataIndex]=RxBuffer;
				DataIndex++;
				ReceiveState = 3;
				}
			else {ReceiveState = 0;}
		}
		else if (RxBuffer>='0' && RxBuffer<='9')
		{
		    ReceiveData[DataIndex]=RxBuffer;
		    DataIndex++;
			if (DataIndex > MaxLength) {ReceiveState = 0;}
			else {ReceiveState = 3;}
		}
		else if (RxBuffer==TelecommandStart)
		{
			ReceiveState=1;
		}
		else if (RxBuffer==TelecommandStop)
		{
			ReceiveData[DataIndex]= 0x00;
			success=Command(TelecommandID);
			ReceiveState=0;
		}
		else { ReceiveState=0;}
		break;
	default:
		ReceiveState=0;
		break;
	}
	return success;
}


uint8_t Command(uint8_t TelecommandID)
{
	char string[40];

	switch (TelecommandID){

	case TelemetryID:
		TelecommandData.Telemetry = (bool)(atof(ReceiveData));
		TeleUART.write((char*)TelecommandAck, 1);
		TelecommandDataTopic.publish(TelecommandData);
		return 1;

	case ModeID:
		TelecommandData.SystemMode = (uint8_t)(atof(ReceiveData));
		TeleUART.write((char*)TelecommandAck, 1);
		TelecommandDataTopic.publish(TelecommandData);
	    return 1;

	case AHRSModeID:
		TelecommandData.AHRSMode = (uint8_t)(atof(ReceiveData));
		TeleUART.write((char*)TelecommandAck, 1);
		TelecommandData.TelecommandFlag=1;
		TelecommandDataTopic.publish(TelecommandData);
		TelecommandData.TelecommandFlag=0;
		return 1;

	case MotorSpeedID:
	    TelecommandData.MotorSpeed = (int32_t)(atof(ReceiveData));
		TeleUART.write((char*)TelecommandAck, 1);
		TelecommandDataTopic.publish(TelecommandData);
		return 1;

	case VelocityID:
		TelecommandData.Velocity = (float)(atof(ReceiveData));
		TeleUART.write((char*)TelecommandAck, 1);
		TelecommandDataTopic.publish(TelecommandData);
	    return 1;

	case PositionID:
		TelecommandData.Position = (float)(atof(ReceiveData));
		TeleUART.write((char*)TelecommandAck, 1);
		TelecommandDataTopic.publish(TelecommandData);
	    return 1;

	case ControllerID:
		TelecommandData.Controller = (uint8_t)(atof(ReceiveData));
		TeleUART.write((char*)TelecommandAck, 1);
		return 1;

	case ControllerParameterID:
		TelecommandData.ControllerParameter = (uint8_t)(atof(ReceiveData));
		TeleUART.write((char*)TelecommandAck, 1);
		return 1;

	case ControllerParameterGainID:
		TelecommandData.ControllerParameterGain = (float)(atof(ReceiveData));
		TeleUART.write((char*)TelecommandAck, 1);
		TelecommandDataTopic.publish(TelecommandData);
		TelecommandData.Controller = 0;
		TelecommandData.ControllerParameter = 0;
		TelecommandData.ControllerParameterGain = 0;
		return 1;

	case ThreadPeriodID:
		TelecommandData.ThreadPeriod = (uint64_t)(atof(ReceiveData));
		TeleUART.write((char*)TelecommandAck, 1);
		TelecommandDataTopic.publish(TelecommandData);
		return 1;

	case ServoID:
		 TelecommandData.Servo[0] = (uint8_t)(TelecommandIDNr - '0');
		 TelecommandData.Servo[1] = (float)(atof(ReceiveData));
		 TeleUART.write((char*)TelecommandAck, 1);
		 TelecommandDataTopic.publish(TelecommandData);
		 return 1;

	case PWMResID:
		TelecommandData.PWMRes[0] = (uint8_t)(TelecommandIDNr - '0');
		TelecommandData.PWMRes[1] = (uint32_t)(atof(ReceiveData));
		TeleUART.write((char*)TelecommandAck, 1);
		TelecommandDataTopic.publish(TelecommandData);
		return 1;

	case BearingsID:
		 TelecommandData.Bearings = (float)(atof(ReceiveData));
		 TeleUART.write((char*)TelecommandAck, 1);
		 TelecommandDataTopic.publish(TelecommandData);
		 return 1;

	case ThrustersID:
		TelecommandData.Thrusters = (float)(atof(ReceiveData));
		TeleUART.write((char*)TelecommandAck, 1);
		TelecommandDataTopic.publish(TelecommandData);
		return 1;

	case HWResetID:
		    hwResetAndReboot();
			return 1;

	default:
		return 0;
	}
}

class Telecommand: public Thread {

public:

	Telecommand(const char* name) : Thread(name) {
	}

	void init() {
		TeleUART.init(115200);
		TeleUART.config(UART_PARAMETER_ENABLE_DMA, 1);
	}



	void run(){

		char RxBuffer;

		while (1)
		{
			TeleUART.suspendUntilDataReady();

            TeleUART.read(&RxBuffer,1);

            Decode(RxBuffer);
		}
	}
};

Telecommand Telecommand("Telecommand");
