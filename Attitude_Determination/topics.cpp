/*****************************************************************
topics.cpp

Original Created by: Atheel Redah @ University of Würzburg
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4 + Würzburg Uni Informatik 8 Discovery AddOn Board Version 2.0.
*****************************************************************/

#include "rodos.h"
#include "topics.h"

Topic<sSensorData> SensorDataTopic(-1,"Sensor Data");
Topic<sTelecommandData> TelecommandDataTopic(-1,"Telecommand Data");


