/*
 * test.cpp
 *
 *  Created on: 02.01.2017
 *      Author: tmikschl
 */




#include <rodos.h>
#include "espdriver/esp8266.h"
#include "ESP8266.h"

#define WIFI_DEBUG

evol ESP_t ESP;
ESP_CONN_t* conn;
ESP_Result_t espRes;

HAL_UART *uart_esp8266;
HAL_GPIO *gpio0_esp8266;
HAL_GPIO *enable_esp8266;

UDPMsg esp8266rxMsg;
ESP8266 *esp_singleton;

const char *dhcp_enable = "AT+CWDHCP_CUR=0,1\r\n";

bool check = false;

ESP8266::UpdateThread::UpdateThread(ESP8266 *_esp8266, Fifo<UDPMsg,5> *_txFifo) : Thread("ESP8266 Update Thread",1004,5000) {
	esp8266 = _esp8266;
	txFifo = _txFifo;
}

void ESP8266::UpdateThread::run() {
	this->suspendCallerUntil(END_OF_TIME);
	while(true) {
		ESP_Update(&ESP);
		if (ESP_IsReady(&ESP) == espOK && !esp8266->txFifo.isEmpty()) {
			uint32_t bw;
			UDPMsg msg;
			if(txFifo->get(msg))
				espRes = ESP_CONN_Send(&ESP, conn, msg.data, msg.length, &bw, 0);

		}
		AT(NOW()+100*MILLISECONDS);
	}
}

int ESP_Callback(ESP_Event_t evt, ESP_EventParams_t* params) {
	ESP_CONN_t* conn;
    uint8_t* data;
    switch (evt) {                              /* Check events */
				case espEventIdle:
					/*
					#ifdef WIFI_DEBUG
						PRINTF("Stack is IDLE!\r\n");
					#endif
					*/
					esp_singleton->internal_state = ESP8266::wlan_state_inactive;
					break;
        		case espEventConnActive: {
                    conn = (ESP_CONN_t *)params->CP1;   /* Get connection for event */
					#ifdef WIFI_DEBUG
                    	PRINTF("ESP8266: Connection %d just became active!\r\n", conn->Number);
					#endif
                    break;
                }
                case espEventConnClosed: {
                    conn = (ESP_CONN_t *)params->CP1;   /* Get connection for event */
					#ifdef WIFI_DEBUG
                    	PRINTF("ESP8266: Connection %d was just closed!\r\n", conn->Number);
					#endif
                    	esp_singleton->internal_state = ESP8266::wlan_state_inactive;
                    break;
                }
                case espEventDataReceived: {
					#ifdef WIFI_DEBUG
                		//PRINTF("Recv %d\n",params->UI);
					#endif
                    conn = (ESP_CONN_t *)params->CP1;   /* Get connection for event */
                    data = (uint8_t *)params->CP2;      /* Get data */

                    esp8266rxMsg.length = params->UI;
            		RODOS::memcpy(&esp8266rxMsg.data,data,esp8266rxMsg.length);
            		esp_singleton->recvMessage();

                    break;
                }
                case espEventDataSent:
                    conn = (ESP_CONN_t *)params->CP1;   /* Get connection for event */
                    esp_singleton->writeFinished();
                    /*
					#ifdef WIFI_DEBUG
                    	PRINTF("Data sent conn: %d\r\n", conn->Number);
					#endif
					*/
                    //PRINTF("Close conn resp: %d\r\n", ESP_CONN_Close(&ESP, conn, 0));
                    break;
                case espEventDataSentError:
                    conn = (ESP_CONN_t *)params->CP1;   /* Get connection for event */
					#ifdef WIFI_DEBUG
                    	PRINTF("ESP8266: Data sent error conn: %d\r\n", conn->Number);
					#endif
                    //ESP_CONN_Close(&ESP, conn, 0);
                    break;
        default:
            break;
    }

    return 0;
}



ESP8266::ESP8266(HAL_UART *_uart, HAL_GPIO *_enable, HAL_GPIO *_gpio0) : updatethread(this, &this->txFifo) {
	// External Pointers for pure c interface
	uart_esp8266 = _uart;
	gpio0_esp8266 = _gpio0;
	enable_esp8266 = _enable;

	// internal class pointers
	uart = _uart;
	gpio0=_gpio0;
	enable=_enable;

	esp_singleton = this;


}
ESP_ConnectedStation_t stations[3];
uint16_t sr;

/**
 * Initialize the module.
 * @param _ssid SSID of the wifi to connect to
 * @param _pw password of the wifi (WPA2 is assumed)
 * @return
 */
int ESP8266::init(const char *_ssid, const char *_pw, bool _accessPoint) {

	ssid = _ssid;
	pw = _pw;
    accessPoint = _accessPoint;

	gpio0 ->init(true, 1, 1);
	enable->init(true, 1, 0);
	AT(NOW() + 50* MILLISECONDS);
	enable->setPins(1);

	// First Connection to ESP - default baudrate 115200 (hopefully always)
	if ((espRes = ESP_Init(&ESP, 115200, ESP_Callback)) == espOK) {
		#ifdef WIFI_DEBUG
	    	PRINTF("ESP8266: ESP module init successfully!\r\n");
		#endif
	} else {
		#ifdef WIFI_DEBUG
	    	PRINTF("ESP8266: ESP Init error. Status: %d\r\n", espRes);
		#endif
	    return -1;
	}

	// Configure for 2 Mbit
	if((espRes = ESP_SetUART(&ESP,2000000,0,1)) == espOK) {
		#ifdef WIFI_DEBUG
			PRINTF("ESP8266: Changed Baudrate\r\n");
		#endif
	} else {
		#ifdef WIFI_DEBUG
			PRINTF("ESP8266: Problems changing baudrate: %d\r\n", espRes);
		#endif
		return -1;
	}
	// Enable UART Flow Control
	uart->config(UART_PARAMETER_HW_FLOW_CONTROL,1);

	AT(NOW() + 1* SECONDS);


	// Connect to wifi
	if(!accessPoint) {
		if ((espRes = ESP_STA_Connect(&ESP, ssid, pw, NULL, 0, 1)) == espOK) {
			#ifdef WIFI_DEBUG
				PRINTF("ESP8266: Connected to network\r\n");
			#endif
		} else {
			#ifdef WIFI_DEBUG
				PRINTF("ESP8266: Problems trying to connect to network: %d\r\n", espRes);
			#endif
				return -1;
		}
	} else {
/*
		uart_esp8266->write(dhcp_enable,strlen(dhcp_enable));


		AT(NOW() + 1* SECONDS);
		while(uart_esp8266->isDataReady()) {
			uart_esp8266->getcharNoWait();
		}
		uint32_t ipx = 0x6600A8C0;
		ESP_AP_SetIP(&ESP,(const uint8_t*)&ipx,0,1);*/
		/* Set access point */
		ESP.APConf.Hidden = 0;                  /* Allow AP to be seen in network */
		ESP.APConf.MaxConnections = 3;   /* Allow up to 3 devices connected to AP at a time */
		strcpy((char *)ESP.APConf.SSID, ssid); /* Set AP name */
		strcpy((char *)ESP.APConf.Pass, pw);  /* Set AP password */
		ESP.APConf.Ecn = ESP_Ecn_WPA2_PSK;      /* Set security level */
		/* Set config */
		if ((espRes = ESP_AP_SetConfig(&ESP, (ESP_APConfig_t *)&ESP.APConf, 0, 1)) == espOK) {
			#ifdef WIFI_DEBUG
				PRINTF("ESP8266: Access point settings are set. You may connect to AP now\r\n");
			#endif
		} else {
			#ifdef WIFI_DEBUG
		    	PRINTF("ESP8266: Problems trying to set access point settings: %d\r\n", espRes);
			#endif
		    return -1;
		}
	}
	updatethread.resume();
	AT(NOW() + 1* SECONDS);
	address = (uint32_t)ESP.STAIP;
	/*while(true) {
		AT(NOW() + 1 *SECONDS);
    if ((espRes = ESP_AP_ListConnectedStations(&ESP, stations, 3, &sr, 1)) == espOK) {
        if (sr) {
            uint8_t i = 0;
            PRINTF("%d station(s) found on soft Access Point\r\n", sr);
            for (i = 0; i < sr; i++) {
                PRINTF("Device %d: %d.%d.%d.%d\r\n", i, stations[i].IP[0], stations[i].IP[1], stations[i].IP[2], stations[i].IP[3]);
            }
        } else {
            PRINTF("No stations connected to our Access Point\r\n");
        }
    } else {

    }
	}*/
	return 0;
}

/**
 * Reset the module.
 */
void ESP8266::reset(){};

/**
 * Get current Status of the Wifi module. See wlan_state.
 * @return current status
 */
int ESP8266::status(){
	return (int) internal_state;
};

/**
 * Sent a UDP message.
 * @param msg udp message to send.
 * @return message length if successful, else -1
 */
int ESP8266::write(UDPMsg *msg){
		if(txFifo.put(*msg)){
			updatethread.resume();
			return msg->length;
		}
	return -1;
};

/**
 * Check whether new UDP messages are available.
 * @return true if new messages are available, false if not.
 */
bool ESP8266::isDataReady(){
	return !rxFifo.isEmpty();
};

/**
 * Read a UDP message from the wf121 module and put it in the provided buffer.
 * @param msg Pointer to UDP message for writing.
 * @return 1 if message was written, 0 if no new message is available.
 */
int ESP8266::read(UDPMsg *msg){
	return rxFifo.get(*msg);
};

void ESP8266::recvMessage() {
	if(!rxFifo.put(esp8266rxMsg)) {
		PRINTF("Wifi - Recevive Buffer full\n\r");
	}
	udp_msgs_recv++;
	upCallDataReady();
}

void ESP8266::writeFinished() {
	isbusy = false;
	upCallWriteFinished();
}


/**
 * Enable UDP Connection. The UDP Connection enabled here,
 * can be accessed via the read and write methods of this class.
 * If the wifi is not ready yet, the udp connection will
 * be activated as soon as the connection is ready.
 *
 * @param _udp_destination destination of the udp connection.
 * Reverse ip in hex format - for example 192.168.0.1 = 0x0100A8C0.
 * This can be a broadcast address, for example 0xFFFFFFFF.
 * Warning: in case of broadcast expect a higher packet loss rate.
 *
 * @param _udp_port UDP port of the local udp server as well of the remote udp connection.
 */
void ESP8266::enableUDPConnection(const char *_udp_destination, uint32_t _udp_port){
	udp_destination = _udp_destination;
	udp_port = _udp_port;


	if ((espRes = ESP_CONN_Start(&ESP, &conn, ESP_CONN_Type_UDP, udp_destination, udp_port, 1)) == espOK) {
		if(ESP.STAIP[0] != 0)
			esp_singleton->internal_state = ESP8266::wlan_state_connect;
		#ifdef WIFI_DEBUG
			PRINTF("ESP8266: UDP Server mode is enabled. IP: %d.%d.%d.%d\r\n", ESP.STAIP[0], ESP.STAIP[1], ESP.STAIP[2], ESP.STAIP[3]);
		#endif
	} else {
		#ifdef WIFI_DEBUG
	    	PRINTF("ESP8266: Problems trying to enable server mode: %d\r\n", espRes);
		#endif
	}

};


void ESP8266::enableTCPConnection(const char *_tcp_destination, uint32_t _tcp_port){
	tcp_destination = _tcp_destination;
	tcp_port = _tcp_port;


	if ((espRes = ESP_CONN_Start(&ESP, &conn, ESP_CONN_Type_TCP, _tcp_destination, tcp_port, 1)) == espOK) {
		#ifdef WIFI_DEBUG
			PRINTF("ESP8266: TCP Connection is enabled. IP: %d.%d.%d.%d\r\n", ESP.APIP[0],ESP.APIP[1],ESP.APIP[2],ESP.APIP[3]); //.STAIP[0], ESP.STAIP[1], ESP.STAIP[2], ESP.STAIP[3]);
		#endif
	} else {
		#ifdef WIFI_DEBUG
	    	PRINTF("ESP8266: Problems trying to enable tcp connection: %d\r\n", espRes);
		#endif
	}
	PRINTF("TCP is not implemented for this driver!");

};



void ESP8266::ESPUpdateTimeEvent::handle(){
  ESP_UpdateTime(&ESP, 1);
}

void ESP8266::ESPUpdateTimeEvent::init() {
  activatePeriodic(1*MILLISECONDS, 1*MILLISECONDS);
}



