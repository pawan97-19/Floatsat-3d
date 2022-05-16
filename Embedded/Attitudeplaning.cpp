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

CommBuffer<Telecommandmsg> TelecommandDataBuffer4;
Subscriber TelecommandDataSubscriber4(TelecommandDataTopic, TelecommandDataBuffer4);
// ------------------------------------------------
CommBuffer<ImuDatamsg> IMUDataBuffer4;
Subscriber IMUDataSubscriber4(ImuTopic, IMUDataBuffer4);


class Attitudeplaning: public Thread {

private:
	Telecommandmsg TelecommandDataReceiver1;
	ImuDatamsg ImuDataReceiver1;
	APmsg AP2Telemetrie;

public:

	Attitudeplaning(const char* name) : Thread(name) {

	}

	void init(void){

	}

	void run() {

		this->TelecommandDataReceiver1.ThreadPeriod[4] = 1500;

		for(int i = 0;i<6;i++){
			if(i==2 || i==3){
				this->TelecommandDataReceiver1.goal_orientation[i] = 1.0;
			}else{
				this->TelecommandDataReceiver1.goal_orientation[i] = 0.0;}
		}

		for(int i = 0;i<6;i++){
			if(i==2 || i==3){
				this->ImuDataReceiver1.orientation[i] = 1.0;
			}else{
				this->ImuDataReceiver1.orientation[i] = 0.0;}
		}

		for(int i = 0;i<3;i++){
			this->AP2Telemetrie.euler[i] = 0.0;
		}

		uint64_t thread_period = 6;

		init();

		while (1) {
			TelecommandDataBuffer4.getOnlyIfNewData(this->TelecommandDataReceiver1);
			IMUDataBuffer4.getOnlyIfNewData(this->ImuDataReceiver1);
			thread_period = this->TelecommandDataReceiver1.ThreadPeriod[4] == -1? thread_period:this->TelecommandDataReceiver1.ThreadPeriod[4];

			attitudeplanning_Run(this->TelecommandDataReceiver1.goal_orientation, this->ImuDataReceiver1.orientation, this->AP2Telemetrie.euler);

			AttitudeplanningTopic.publish(this->AP2Telemetrie);
			suspendCallerUntil(NOW() + thread_period*MILLISECONDS);
		}
	}
};
Attitudeplaning Attitudeplaning("Attitudeplaning");

/***********************************************************************/
