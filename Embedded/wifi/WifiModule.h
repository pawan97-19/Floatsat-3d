/*
 * WifiModule.h
 *
 *  Created on: Jun 20, 2018
 *      Author: info8
 */

#pragma once
#include "rodos.h"

#include "drivers/esp8266/ESP8266.h"

// Wifi Gateway

extern RODOS::HAL_UART uart3;
extern RODOS::HAL_GPIO wifi_gpio0;
extern RODOS::HAL_GPIO wifi_en;
extern ESP8266 wf;
//extern LinkinterfaceWifi linkw;
//extern Gateway gw;

