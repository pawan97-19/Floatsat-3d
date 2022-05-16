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

CommBuffer<Telecommandmsg> TelecommandDataBuffer1;
Subscriber TelecommandDataSubscriber1(TelecommandDataTopic, TelecommandDataBuffer1);
// ------------------------------------------------
CommBuffer<ImuDatamsg> IMUDataBuffer2;
Subscriber IMUDataSubscriber2(ImuTopic, IMUDataBuffer2);


class CMrelocestimation: public Thread {

	Telecommandmsg TelecommandDataReceiver4;
	ImuDatamsg ImuDataReceiver4;

	CM2Telemetrie satparamMsg4;

public:

	CMrelocestimation(const char* name) : Thread(name) {

	}

	void init() {
		TelecommandDataReceiver4.ThreadPeriod[1] = 2000;

		for(int i = 0;i<3;i++){
			ImuDataReceiver4.acceleration[i] = 0.0;
		}

		for(int i = 0;i<3;i++){
			ImuDataReceiver4.Gyro[i] = 0.0;
		}

		for(int i = 0;i<3;i++){
			ImuDataReceiver4.Magneto[i] = 0.0;
		}
	}

	void run() {

		uint64_t thread_period = 1000;

		while (1) {
			TelecommandDataBuffer1.getOnlyIfNewData(TelecommandDataReceiver4);
			IMUDataBuffer2.getOnlyIfNewData(ImuDataReceiver4);
			thread_period = TelecommandDataReceiver4.ThreadPeriod[1] == -1? thread_period:TelecommandDataReceiver4.ThreadPeriod[1];



			CM2TelemetrieTopic.publish(satparamMsg4);
			suspendCallerUntil(NOW() + thread_period*MILLISECONDS);
		}
	}
};
CMrelocestimation CMrelocestimation("CMrelocestimation");

/***********************************************************************/
