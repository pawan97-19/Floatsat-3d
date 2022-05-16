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

		this->TelecommandDataReceiver1.ThreadPeriod[4] = 110;
		this->TelecommandDataReceiver1.SystemMode = 1;

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

		uint64_t thread_period = 205;

		double rotmat[9] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

		while (1) {

			if(IMUDataBuffer4.getOnlyIfNewData(this->ImuDataReceiver1) && this->TelecommandDataReceiver1.SystemMode == 1){

				thread_period = this->TelecommandDataReceiver1.ThreadPeriod[4] == -1? thread_period:this->TelecommandDataReceiver1.ThreadPeriod[4];

				eul2RotMat(this->ImuDataReceiver1.orientation, rotmat);

				double ZX_actual_orientation[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
				double mat_1[3][3] = {{rotmat[0],rotmat[1],rotmat[2]},{rotmat[3],rotmat[4],rotmat[5]},{rotmat[6],rotmat[7],rotmat[8]}};
				double mat_2[3][2] = {{0.0,1.0},{0.0,0.0},{1.0,0.0}};
				matrix_mult(mat_1,mat_2,3,3,2 ,ZX_actual_orientation);

				attitudeplanning_Run(this->TelecommandDataReceiver1.goal_orientation, ZX_actual_orientation, this->AP2Telemetrie.euler);
				AttitudeplanningTopic.publish(this->AP2Telemetrie);

				PRINTF("actual orientation : Zx = %f, Zy = %f, Zz = %f,Xx = %f, Xy = %f, Xz = %f \r\n", ZX_actual_orientation[0],
						ZX_actual_orientation[1],ZX_actual_orientation[2],ZX_actual_orientation[3],ZX_actual_orientation[4],
						ZX_actual_orientation[5]);
			}

			suspendCallerUntil(NOW() + thread_period*MILLISECONDS);
		}
	}
};
Attitudeplaning Attitudeplaning("Attitudeplaning");

/***********************************************************************/
