
/*
 * Initialization.cpp
 *
 *  Created on: Jun 20, 2018
 *      Author: info8
 */

#include "WifiModule.h"
#include "stdio.h"
#include "..\topics.h"

#define SSID			"LAPTOP-MST2KV5N 4395"
#define PASSWORD		"67{y46T2"
#define CU1_IP 		    "192.168.137.1" // from Ground
#define CU1_PORT		 21560 // Port to read the data from Ground

#define CU2_IP 		    "192.168.0.5" // to Ground
#define CU2_PORT		 21566 // Port to read the data from floatsat

UDPMsg TelemetryMsg;
Telecommandmsg TelecommandDataReceiver5; //input
AC2Telemetrie ACmsg5; //input
APmsg AP2Telemetrie5; //input
CM2Telemetrie satparamMsg5; //input
ImuDatamsg Imudatacollector5; //input

CommBuffer<APmsg> APDataBuffer;
Subscriber APDataSubscriber(AttitudeplanningTopic, APDataBuffer); // for the attitude planning

CommBuffer<ImuDatamsg> IMUDataBuffer;
Subscriber IMUDataSubscriber(ImuTopic, IMUDataBuffer);  // for the IMU

CommBuffer<AC2Telemetrie> ACDataBuffer;
Subscriber ACDataSubscriber(AC2TelemetrieTopic, ACDataBuffer); // for the Attitude Control

CommBuffer<CM2Telemetrie> CMDataBuffer;
Subscriber CMDataSubscriber(CM2TelemetrieTopic, CMDataBuffer);

CommBuffer<Telecommandmsg> TelecommandDataBuffer;
Subscriber TelecommandDataSubscriber(TelecommandDataTopic, TelecommandDataBuffer);

class SendingTelemetry :public Thread {

public:

	void sendtelemetry2wifi(){

/*		sprintf((char*)TelemetryMsg.data, "Mode = %d ;",(int)TelecommandDataReceiver5.SystemMode);
		TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
		wf.write(&TelemetryMsg);*/

		sprintf((char*)TelemetryMsg.data, "Time = %d ;",(int)SECONDS_NOW());
		TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
		wf.write(&TelemetryMsg);

/*		sprintf((char*)TelemetryMsg.data, "Orientation = [%f,%f,%f,%f,%f,%f] ;",Imudatacollector5.orientation[0],
				Imudatacollector5.orientation[1],Imudatacollector5.orientation[2],Imudatacollector5.orientation[3],
				Imudatacollector5.orientation[4],Imudatacollector5.orientation[5]);
		TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
		wf.write(&TelemetryMsg);*/

		sprintf((char*)TelemetryMsg.data, "acceleration = [%f,%f,%f] ;",Imudatacollector5.acceleration[0],
				Imudatacollector5.acceleration[1],Imudatacollector5.acceleration[2]);
		TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
		wf.write(&TelemetryMsg);

		sprintf((char*)TelemetryMsg.data, "Gyro = [%f,%f,%f] ;",Imudatacollector5.Gyro[0],Imudatacollector5.Gyro[1],
				Imudatacollector5.Gyro[2]);
		TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
		wf.write(&TelemetryMsg);

		sprintf((char*)TelemetryMsg.data, "Magneto = [%f,%f,%f] ;",Imudatacollector5.Magneto[0],Imudatacollector5.Magneto[1],
				Imudatacollector5.Magneto[2]);
		TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
		wf.write(&TelemetryMsg);

		/*sprintf((char*)TelemetryMsg.data, "DCmotor = [%d,%d,%d] ;",ACmsg5.DCSpeed[0],ACmsg5.DCSpeed[1],ACmsg5.DCSpeed[2]);
		TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
		wf.write(&TelemetryMsg);

		sprintf((char*)TelemetryMsg.data, "STmotor = [%d,%d,%d] ;",ACmsg5.STSpeed[0],ACmsg5.STSpeed[1],ACmsg5.STSpeed[2]);
		TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
		wf.write(&TelemetryMsg);

		sprintf((char*)TelemetryMsg.data, "MMpose = [%f,%f,%f] ;",ACmsg5.Mmpose[0],ACmsg5.Mmpose[1],ACmsg5.Mmpose[2]);
		TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
		wf.write(&TelemetryMsg);

		sprintf((char*)TelemetryMsg.data, "Next Move = [%f,%f,%f] ;",AP2Telemetrie5.euler[0],AP2Telemetrie5.euler[1],
		AP2Telemetrie5.euler[2]);
		TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
		wf.write(&TelemetryMsg);

		sprintf((char*)TelemetryMsg.data, "estimated Satellite Parameters = [%f,%f,%f] ;",satparamMsg5.Sat_param[0],
		satparamMsg5.Sat_param[1]
																																		   ,satparamMsg5.Sat_param[2],satparamMsg5.Sat_param[3],satparamMsg5.Sat_param[4],satparamMsg5.Sat_param[5],satparamMsg5.Sat_param[6],
																																		   satparamMsg5.Sat_param[7],satparamMsg5.Sat_param[8]);
		TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
		wf.write(&TelemetryMsg);
		// satparamMsg*/
	}

	void init (void) {

	}

	void run(void) {

		wf.init(SSID, PASSWORD);
		wf.enableUDPConnection(CU1_IP, CU1_PORT);
		TelecommandDataReceiver5.WiFi = 1;
		TelecommandDataReceiver5.SystemMode = 0;

		while(1) {

			uint64_t thread_period = 10;
			TelecommandDataBuffer.getOnlyIfNewData(TelecommandDataReceiver5);
			APDataBuffer.getOnlyIfNewData(AP2Telemetrie5);
			IMUDataBuffer.getOnlyIfNewData(Imudatacollector5);
			ACDataBuffer.getOnlyIfNewData(ACmsg5);
			CMDataBuffer.getOnlyIfNewData(satparamMsg5);
			thread_period = TelecommandDataReceiver5.ThreadPeriod[0] == -1? thread_period:TelecommandDataReceiver5.ThreadPeriod[0];


			if (TelecommandDataReceiver5.WiFi == 1)
			{
				wf.enableUDPConnection(CU1_IP, CU1_PORT);
				TelecommandDataReceiver5.WiFi = 0;
			}

			if (TelecommandDataReceiver5.WiFi == 2)
			{
				wf.enableUDPConnection(CU2_IP, CU2_PORT);
				TelecommandDataReceiver5.WiFi = 0;
			}

			else if (TelecommandDataReceiver5.WiFi == 255)  // Reset the WiFi Module
			{
				wifi_en.setPins(0);
				AT(NOW()+100*MILLISECONDS);
				wifi_en.setPins(1);
				wf.init(SSID, PASSWORD);
				wf.enableUDPConnection(CU1_IP, CU1_PORT);
				TelecommandDataReceiver5.WiFi = 0;
			}

			sendtelemetry2wifi();
			suspendCallerUntil(NOW() + thread_period*MILLISECONDS);
		} // end time loop
	} // run ends
} sendTelemetry;
