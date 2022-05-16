/*
 * Wifi.h
 *
 *  Created on: 18.01.2017
 *      Author: tmikschl
 */

#pragma once

#include <rodos.h>
#include <stdio.h>

// See includes at the end of the file !!!

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

typedef struct {
	int16_t length;
	uint8_t data[255];
}UDPMsg;

class HAL_Wifi  : public GenericIOInterface {
protected:
	uint32_t address;
	uint32_t gateway;
	uint32_t netmask;
	uint8_t dhcp;
	const char *pw;
	const char *ssid;
	uint32_t udp_msgs_send;
	uint32_t udp_msgs_recv;
	bool accessPoint;

public:	
	HAL_Wifi(){ }
	virtual ~HAL_Wifi() { }

/**
		 * Initialize the module.
		 * @param _ssid SSID of the wifi to connect to
		 * @param _pw password of the wifi (WPA2 is assumed)
		 * @param
		 * @return
		 */
		virtual int init(const char *_ssid, const char *_pw, bool _accessPoint = false) { return 0; };

		/**
		 * Reset the module.
		 */
		virtual void reset() {};

		/**
		 * Get current Status of the Wifi module. See wlan_state.
		 * @return current status
		 */
		virtual int status() { return 0; }

		/**
		 * Sent a UDP message.
		 * @param msg udp message to send.
		 * @return message length if successful, else -1
		 */
		virtual int write(UDPMsg *msg) {return -1; }

		/**
		 * Check whether new UDP messages are available.
		 * @return true if new messages are available, false if not.
		 */
		virtual bool isDataReady(){ return false; }

		/**
		 * Read a UDP message from the wf121 module and put it in the provided buffer.
		 * @param msg Pointer to UDP message for writing.
		 * @return 1 if message was written, 0 if no new message is available.
		 */
		virtual int read(UDPMsg *msg) { return 0; }



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
		virtual void enableUDPConnection(const char *_udp_destination, uint32_t _udp_port) {};


		virtual void enableTCPConnection(const char *_tcp_destination, uint32_t _tcp_port) {};

		/**
		 * Get current ip address of the module.
		 * @return current ip address.
		 */
		uint32_t getAddress() { return address; };

		/**
		 * Return current gateway.
		 * @return current gateway.
		 */
		uint32_t getGateway() { return gateway; };

		/**
		 * Return current netmask.
		 * @return current netmask.
		 */
		uint32_t getNetmask() { return netmask; };

		/**
		 * Return whether dhcp was used to obtain ip address or not.
		 * @return 1 if dhcp was used, 0 if not.
		 */
		uint8_t getDHCP() { return dhcp; };

		/**
		 * Return currently configured SSID.
		 * @return SSID of the wifi network.
		 */
		const char *getSSID() { return ssid; };

		/**
		 * Return number of received udp messages.
		 * @return number of messages
		 */
		uint32_t getUDPMsgsRecv() { return udp_msgs_recv; };

		/**
		 * Return number of send messages.
		 * @return number of messages
		 */
		uint32_t getUDPMsgsSend() { return udp_msgs_send; };

		/**
		 * Converts IP from uint32_t representation to string
		 * @param buf buffer fpor string
		 * @param ip ip address as reversed int32 - for example 192.168.0.1 = 0x0100A8C0.
		 */
		void convertIPtoString(char *buf,uint32_t ip) {
			sprintf(buf,"%d.%d.%d.%d",(uint8_t)ip,(uint8_t)(ip>>8),(uint8_t)(ip>>16),(uint8_t)(ip>>24));
		}

		/**
		 * Converts IP to uint32_t representation to string
		 * @param buf buffer fpor string
		 * @param ip ip address as reversed int32 - for example 192.168.0.1 = 0x0100A8C0.
		 */
		uint32_t convertIPfromString(const char *buf) {
			uint32_t ip;
			sscanf(buf,"%c.%c.%c.%c",((char*)&ip) + 0, ((char*)&ip) + 1, ((char*)&ip) + 2, ((char*)&ip) + 3);
			return ip;
		}
};

#ifndef NO_RODOS_NAMESPACE
}
#endif



