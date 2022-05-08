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

uint32_t TelemetryPeriod = 1000; // Telemetry period in ms

#define LED_BLUE GPIO_060    //PD12
#define LED_RED GPIO_061  //PD13
#define LED_ORANGE GPIO_062  //PD14
#define LED_GREEN GPIO_063   //PD15

namespace RODOS {
extern HAL_UART uart_stdout;
}
#define TeleUART uart_stdout


//HAL_GPIO TelemetryLED(LED_GREEN);

CommBuffer<sSensorData> SensorDataBuffer;
Subscriber SensorDataSubscriber(SensorDataTopic, SensorDataBuffer);

CommBuffer<sTelecommandData> TelecommandDataBuffer;
Subscriber TelecommandDataSubscriber(TelecommandDataTopic, TelecommandDataBuffer);


class Telemetry: public Thread {

	uint64_t periode;
	sSensorData SensorDataReceiver;
	sTelecommandData TelecommandDataReceiver;

public:

	Telemetry(const char* name, uint64_t periode) : Thread(name) {
			this->periode = periode;
		}

	void init() {
		//TelemetryLED.init(true, 1, 0);
	}

	void write(const char *buf, int size){
	    int sentBytes=0;
	    int retVal;
	    while(sentBytes < size){
	        retVal = TeleUART.write(&buf[sentBytes],size-sentBytes);
	        if (retVal < 0){
	            PRINTF("UART sent error\n");
	        }else{
	            sentBytes+=retVal;
	        }
	    }
	}


	void run() {



		TIME_LOOP(0, periode){

					//TelemetryLED.setPins(~TelemetryLED.readPins());

					SensorDataBuffer.get(SensorDataReceiver);

				/*	RangeDataBuffer.get(RangeDataReceiver);

					LightDataBuffer.get(LightDataReceiver);*/

					if(TelecommandDataBuffer.getOnlyIfNewData(TelecommandDataReceiver))
					{
						if (TelecommandDataReceiver.ThreadPeriod > 0)
						{setPeriodicBeat(0, TelecommandDataReceiver.ThreadPeriod * MILLISECONDS);}
					}


		       PRINTF("Hello Rodos, the time now is %f \r\n",SECONDS_NOW());

		       PRINTF("Motor Speed is %f and System Mode is %d \r\n",SensorDataReceiver.motorSpeed,TelecommandDataReceiver.SystemMode);


		}
	}
};
Telemetry Telemetry("Telemetry", TelemetryPeriod * MILLISECONDS);

/***********************************************************************/
