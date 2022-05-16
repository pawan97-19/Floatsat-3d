
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
char Sip[16] = "192.168.137.1";

Telecommandmsg getTelecommandData = {
		{0000,0000,0000}, // DC Speed
		{0000,0000,0000}, // ST pose
		{0.0,0.0,1.0,0.0,0.0,1.0}, // goal Orientation
		0, // Stop all
		0, // valid Telecommands
		0, // estimate CM
		0, // relocate CM
		-1, // Calibrate IMU
		{70,150,50,90,110}, // Threads Periods
		1, // Wifi
		0, // System Mode
		{0.0062,0.0062,0.0062}, // Is
		{0.0011,0.0011,0.0011}, // Irw
		8, // filter Mode
		{0.0,0.0,0.0}, // moving masses next pose
		21566,
		{0},
		{1,1,1,1,0,0}
};

class Receiver : public Thread {
	char* userData;

	void init (void) {

	}

	void run () {

		while(1) {
			TelecommandDataTopic.publish(getTelecommandData);
			wf.suspendUntilDataReady();
			wf.read(&TeleCommandMsg_sock);
			userData=(char*)TeleCommandMsg_sock.data;
			PRINTF("%s\r\n",userData);
			ExtractData(userData,&getTelecommandData);
			PRINTF("User Mode is : %d \r\n",getTelecommandData.SystemMode);

			TelecommandDataTopic.publish(getTelecommandData);
			//suspendCallerUntil(NOW()+151*MILLISECONDS);
		} // loop
	} // run
} recieve;


