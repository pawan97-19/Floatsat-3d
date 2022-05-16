/*
 * WifiModule.cpp
 *
 *  Created on: Jun 20, 2018
 *      Author: info8
 */

#include"WifiModule.h"

// Wifi Gateway
RODOS::HAL_UART uart3(UART_IDX3);
RODOS::HAL_GPIO wifi_gpio0(GPIO_028);
RODOS::HAL_GPIO wifi_en(GPIO_008);
ESP8266 wf(&uart3,&wifi_en,&wifi_gpio0);

RODOS::HAL_GPIO led_B(GPIO_060); 		// Discovery: LED blue
