
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
#define CU1_IP 		    "10.107.150.84" // Ground IP
#define CU1_PORT		 21566 // Port to read the data from Ground

#define CU2_IP 		    "192.168.2.154" // to Ground
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

		if(TelecommandDataBuffer.getOnlyIfNewData(TelecommandDataReceiver5)){
			if(TelecommandDataReceiver5.flo2ground[1] == 1){
				sprintf((char*)TelemetryMsg.data, "Mode=%d",TelecommandDataReceiver5.SystemMode);
				TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
				wf.write(&TelemetryMsg);
				AT(NOW()+4*MILLISECONDS);
			}
		}

		if(TelecommandDataReceiver5.flo2ground[0] == 1){

			// will be send constantly
			sprintf((char*)TelemetryMsg.data, "Time=%d",(int)SECONDS_NOW());
			TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
			wf.write(&TelemetryMsg);
			AT(NOW()+4*MILLISECONDS);
		}

		if(IMUDataBuffer.getOnlyIfNewData(Imudatacollector5)){
			if(TelecommandDataReceiver5.flo2ground[3] == 1){
				sprintf((char*)TelemetryMsg.data, "<Orientation=[%f,%f,%f]>",Imudatacollector5.orientation[0],
						Imudatacollector5.orientation[1],Imudatacollector5.orientation[2]);
				TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
				wf.write(&TelemetryMsg);
				AT(NOW()+4*MILLISECONDS);
			}

			if(TelecommandDataReceiver5.flo2ground[2] == 1){
				sprintf((char*)TelemetryMsg.data, "<Acceleration=[%f,%f,%f]>",Imudatacollector5.acceleration[0],
						Imudatacollector5.acceleration[1],Imudatacollector5.acceleration[2]);
				TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
				wf.write(&TelemetryMsg);
				AT(NOW()+4*MILLISECONDS);

				sprintf((char*)TelemetryMsg.data, "<Gyro=[%f,%f,%f]>",Imudatacollector5.Gyro[0],Imudatacollector5.Gyro[1],
						Imudatacollector5.Gyro[2]);
				TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
				wf.write(&TelemetryMsg);
				AT(NOW()+4*MILLISECONDS);

				sprintf((char*)TelemetryMsg.data, "<Magneto=[%f,%f,%f]>",Imudatacollector5.Magneto[0],Imudatacollector5.Magneto[1],
						Imudatacollector5.Magneto[2]);
				TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
				wf.write(&TelemetryMsg);
				AT(NOW()+4*MILLISECONDS);
			}
		}

		if(ACDataBuffer.getOnlyIfNewData(ACmsg5)){
			if(TelecommandDataReceiver5.flo2ground[4] == 1){
				sprintf((char*)TelemetryMsg.data, "DCmotor=[%d,%d,%d]",ACmsg5.DCSpeed[0],ACmsg5.DCSpeed[1],ACmsg5.DCSpeed[2]);
				TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
				wf.write(&TelemetryMsg);
				AT(NOW()+4*MILLISECONDS);
			}
				sprintf((char*)TelemetryMsg.data, "NextMove=[%f,%f,%f]",AP2Telemetrie5.euler[0],AP2Telemetrie5.euler[1],
						AP2Telemetrie5.euler[2]);
				TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
				wf.write(&TelemetryMsg);

		}

		if(TelecommandDataReceiver5.flo2ground[5] == 1){
			sprintf((char*)TelemetryMsg.data, "STmotor=[%d,%d,%d]",ACmsg5.STSpeed[0],ACmsg5.STSpeed[1],ACmsg5.STSpeed[2]);
			TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
			wf.write(&TelemetryMsg);
			AT(NOW()+4*MILLISECONDS);

			sprintf((char*)TelemetryMsg.data, "MMpose=[%f,%f,%f]",ACmsg5.Mmpose[0],ACmsg5.Mmpose[1],ACmsg5.Mmpose[2]);
			TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
			wf.write(&TelemetryMsg);
			AT(NOW()+4*MILLISECONDS);
		}

		/*		if(CMDataBuffer.getOnlyIfNewData(satparamMsg5)){
			sprintf((char*)TelemetryMsg.data, "eSatParam=[%f,%f,%f,%f,%f,%f,%f,%f,%f]",satparamMsg5.Sat_param[0],
					satparamMsg5.Sat_param[1],satparamMsg5.Sat_param[2],satparamMsg5.Sat_param[3],satparamMsg5.Sat_param[4],
					satparamMsg5.Sat_param[5],satparamMsg5.Sat_param[6],satparamMsg5.Sat_param[7],satparamMsg5.Sat_param[8]);
			TelemetryMsg.length = strlen((char*)TelemetryMsg.data);
			wf.write(&TelemetryMsg);
			AT(NOW()+4*MILLISECONDS);
			// satparamMsg
		}*/
	}

	void init (void) {
	}

	void run(void) {

		TelecommandDataReceiver5.ThreadPeriod[0] = -1;
		TelecommandDataReceiver5.WiFi = 1;
		TelecommandDataReceiver5.SystemMode = 0;
		wf.init(SSID, PASSWORD);
		wf.enableUDPConnection(CU1_IP, CU1_PORT);

		uint64_t thread_period = 300;

		while(1) {

			sendtelemetry2wifi();
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

			else if (TelecommandDataReceiver5.WiFi == 3)  // Reset the WiFi Module
			{
				wifi_en.setPins(0);
				AT(NOW()+100*MILLISECONDS);
				wifi_en.setPins(1);
				wf.init(SSID, PASSWORD);
				wf.enableUDPConnection(CU1_IP, CU1_PORT);
				TelecommandDataReceiver5.WiFi = 0;
			}

			suspendCallerUntil(NOW() + thread_period*MILLISECONDS);
			//suspendUntilNextBeat();
		} // end time loop
	} // run ends
} sendTelemetry;
