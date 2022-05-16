
/*
 * Initialization.cpp
 *
 *  Created on: Jun 20, 2018
 *      Author: info8
 */

#include "WifiModule.h"
#include"DataExtraction.h"
#include <stdio.h>

static Application module01("Floatsat", 2022);

UDPMsg TeleCommandMsg_sock;

Telecommandmsg getTelecommandData;

class Receiver : public Thread {
	char* userData;

	void run () {
		while(1) {

			wf.suspendUntilDataReady();
			wf.read(&TeleCommandMsg_sock);
			userData=(char*)TeleCommandMsg_sock.data;
			PRINTF(userData);
			ExtractData(userData,getTelecommandData);

			/*
			 * depending on the system mode check which thread should be on/off.
			 *
			 * 	SystemMode: 1 for Validation, 2 for all Calibration, 3 for GYRO Calibration, 4 for ACC Calibration, 5 for MAG Calibration
			 * 	6 for CM estimation, 7 for CM Relocation, 8 for Orientation, 9 for DC control, 10 for ST control, 11 for all control, 12 for stop all.
			 *
			 * */


			TelecommandDataTopic.publish(getTelecommandData);
			suspendCallerUntil(NOW()+500*MILLISECONDS);
		} // loop
	} // run
} recieve;


