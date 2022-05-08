/*
 *Balancer.cpp
 *
 *  Created on: Mar 11, 2022
 *      Author: Atheel Redah
 */

#include "rodos.h"
#include "topics.h"

#define SPI_STEPPER_CS1   GPIO_053		// PD5
#define SPI_STEPPER_CS2   GPIO_054	    // PD6
#define SPI_STEPPER_CS3   GPIO_027		// PB11

#define SPI_STEPPER       SPI_IDX3
#define SPI_STEPPER_SCK   GPIO_042		// PC10
#define SPI_STEPPER_MISO  GPIO_043		// PC11
#define SPI_STEPPER_MOSI  GPIO_044		// PC12

#define L6470_CONFIG 	0x18
#define dSPIN_GET_PARAM 0x20
#define dSPIN_FORWARD	0x50
#define dSPIN_REVERSE	0x51

void set_stepper_speed(int32_t speed, int stepper_num, float direction);
void Find_Speed_Duration_Direction(uint8_t StepperNumber, uint8_t GoTo);

const int max_speed = 0x103FF; //maxiumum speed of the stepper motor according to data sheet = 1048575

uint32_t BalancerPeriod = 300; // Thread period in ms

enum Stepper
{
	Stepper1    = 1,
	Stepper2    = 2,
	Stepper3    = 3,
};

int64_t MAXDISTANCE = 50; //40mm
uint8_t distance1 = 0;
uint8_t distance2 = 0;
uint8_t distance3 = 0;
int32_t speed1;
int32_t speed2;
int32_t speed3;
float Duration1;
float Duration2;
float Duration3;
uint8_t Direction1;
uint8_t Direction2;
uint8_t Direction3;
uint8_t Stepper;

CommBuffer<sTelecommandData> BalancerTelecommandDataBuffer;
Subscriber BalancerTelecommandDataSubscriber(TelecommandDataTopic, BalancerTelecommandDataBuffer);

HAL_GPIO stepper_cs(SPI_STEPPER_CS1);
HAL_GPIO stepper_cs1(SPI_STEPPER_CS2);
HAL_GPIO stepper_cs2(SPI_STEPPER_CS3);
HAL_SPI stepper_spi(SPI_STEPPER,SPI_STEPPER_SCK, SPI_STEPPER_MISO, SPI_STEPPER_MOSI);

void Find_Speed_Duration_Direction(uint8_t StepperNumber, uint8_t GoTo) {
	uint8_t distance;
	switch (StepperNumber) {
	case 1:
		distance = abs(GoTo - distance1);
		Direction1 = (GoTo > distance1) ? dSPIN_FORWARD : dSPIN_REVERSE;
		distance1 = GoTo;

		if (distance < 5) {
			speed1 = 10300;
			Duration1 = distance * 0.25;
		} else if (distance < 13) {
			speed1 = 20500;
			Duration1 = distance * 0.125;

		} else {
			speed1 = 66000;
			Duration1 = distance * 0.03875;

		}

		break;
	case 2:
		distance = abs(GoTo - distance2);
		Direction2 = (GoTo > distance2) ? dSPIN_FORWARD : dSPIN_REVERSE;
		distance2 = GoTo;
		if (distance < 4) {
			speed2 = 10300;
			Duration2 = distance * 0.25;

		} else if (distance < 13) {
			speed2 = 20500;
			Duration2 = distance * 0.125;

		} else {
			speed2 = 66000;
			Duration2 = distance * 0.03875;
		}
		break;
	case 3:
		distance = abs(GoTo - distance3);
		Direction3 = (GoTo > distance3) ? dSPIN_FORWARD : dSPIN_REVERSE;
		distance3 = GoTo;
		if (distance < 5) {
			speed3 = 10700;
			Duration3 = distance * 0.25;
		} else if (distance < 13) {
			speed3 = 21400;
			Duration3 = distance * 0.125;
		} else {
			speed3 = 66000;
			Duration3 = distance * 0.04100625;
		}
		break;
	}

}

bool Steppper1 = false, Steppper2 = false, Steppper3 = false;
void Move(float Duration1, float Duration2, float Duration3, float dir1,
		float dir2, float dir3, uint8_t speed1, uint8_t speed2, uint8_t speed3 ) {
	if (Duration1 > 0) {
		Steppper1 = true;
		set_stepper_speed(speed1, 1, dir1);
	}
	if (Duration2 > 0) {
		Steppper2 = true;
		set_stepper_speed(speed2, 2, dir2);
	}
	if (Duration3 > 0) {
		Steppper3 = true;
		set_stepper_speed(speed3, 3, dir3);

	}

	while (1) {
		AT(NOW() + 0.25*SECONDS);

		Duration1--;
		if (Duration1 <= 0) {
			set_stepper_speed(speed1, 1, dir1);
			Steppper1 = false;
		}
		Duration2--;
		if (Duration2 <= 0) {
			set_stepper_speed(speed2, 2, dir2);
			Steppper2 = false;
		}
		Duration3--;
		if (Duration3 <= 0) {
			set_stepper_speed(speed3, 3, dir3);
			Steppper3 = false;
		}
		if (Steppper1 == false && Steppper2 == false && Steppper3 == false) {
			break;
		}
	}

}


class Balancer_Thread : public Thread {

	uint64_t periode;
	sTelecommandData TelecommandDataReceiver;

public:

	Balancer_Thread(const char* name, uint64_t periode) : Thread(name) {
		this->periode = periode;
	}

	void init() {

		stepper_cs.init(true, 1, 1);
		stepper_cs1.init(true, 1, 1);
		stepper_cs2.init(true, 1, 1);

		stepper_spi.init(1000000);
	}


	/*
	 *  sets speed in stepper motor via SPI
	 */
	void set_stepper_speed(int32_t speed, int stepper_num, float direction){

		uint8_t data_send[4];
		uint16_t data_rcv;
		uint32_t abs_speed = speed;

		switch (stepper_num){

		case 1:
		// direction definition parameter

			data_send[0] =  direction;



		// 3 bytes of speed parameter
		data_send[1] = abs_speed >> 16;
		data_send[2] = abs_speed >> 8;
		data_send[3] = abs_speed;

		//PRINTF("%x %x %x\n", data_send[1], data_send[2], data_send[3]);

		// write speed via SPI
		stepper_cs.setPins(0);
		stepper_spi.writeRead(&data_send[0],1,(uint8_t *)&data_rcv,1);
		stepper_cs.setPins(1);
		stepper_cs.setPins(0);
		stepper_spi.writeRead(&data_send[1],1,(uint8_t *)&data_rcv,1);
		stepper_cs.setPins(1);
		stepper_cs.setPins(0);
		stepper_spi.writeRead(&data_send[2],1,(uint8_t *)&data_rcv,1);
		stepper_cs.setPins(1);
		stepper_cs.setPins(0);
		stepper_spi.writeRead(&data_send[3],1,(uint8_t *)&data_rcv,1);
		stepper_cs.setPins(1);
		break;

		case 2:
				// direction definition parameter
				if(speed < 0){
					data_send[0] =  dSPIN_REVERSE;
					abs_speed = -speed;
				}
				else{
					data_send[0] =  dSPIN_FORWARD;
					abs_speed = speed;
				}

				// 3 bytes of speed parameter
				data_send[1] = abs_speed >> 16;
				data_send[2] = abs_speed >> 8;
				data_send[3] = abs_speed;

				//PRINTF("%x %x %x\n", data_send[1], data_send[2], data_send[3]);

				// write speed via SPI
				stepper_cs1.setPins(0);
				stepper_spi.writeRead(&data_send[0],1,(uint8_t *)&data_rcv,1);
				stepper_cs1.setPins(1);
				stepper_cs1.setPins(0);
				stepper_spi.writeRead(&data_send[1],1,(uint8_t *)&data_rcv,1);
				stepper_cs1.setPins(1);
				stepper_cs1.setPins(0);
				stepper_spi.writeRead(&data_send[2],1,(uint8_t *)&data_rcv,1);
				stepper_cs1.setPins(1);
				stepper_cs1.setPins(0);
				stepper_spi.writeRead(&data_send[3],1,(uint8_t *)&data_rcv,1);
				stepper_cs1.setPins(1);
				break;

		case 3:
				// direction definition parameter
				if(speed < 0){
					data_send[0] =  dSPIN_REVERSE;
					abs_speed = -speed;
				}
				else{
					data_send[0] =  dSPIN_FORWARD;
					abs_speed = speed;
				}

				// 3 bytes of speed parameter
				data_send[1] = abs_speed >> 16;
				data_send[2] = abs_speed >> 8;
				data_send[3] = abs_speed;

				//PRINTF("%x %x %x\n", data_send[1], data_send[2], data_send[3]);

				// write speed via SPI
				stepper_cs2.setPins(0);
				stepper_spi.writeRead(&data_send[0],1,(uint8_t *)&data_rcv,1);
				stepper_cs2.setPins(1);
				stepper_cs2.setPins(0);
				stepper_spi.writeRead(&data_send[1],1,(uint8_t *)&data_rcv,1);
				stepper_cs2.setPins(1);
				stepper_cs2.setPins(0);
				stepper_spi.writeRead(&data_send[2],1,(uint8_t *)&data_rcv,1);
				stepper_cs2.setPins(1);
				stepper_cs2.setPins(0);
				stepper_spi.writeRead(&data_send[3],1,(uint8_t *)&data_rcv,1);
				stepper_cs2.setPins(1);
				break;
		}
	}

	void Off(int stepper_num){
		set_stepper_speed(0, stepper_num);
	}

    void run () {

		AT(NOW()+5*SECONDS);

		int32_t speed=0;
		uint8_t GoTo;





    	TIME_LOOP(0, periode){

    		BalancerTelecommandDataBuffer.getOnlyIfNewData(TelecommandDataReceiver);
    		Stepper = TelecommandDataReceiver.Stepper;


    		switch (Stepper) {

    		case Stepper1:
    			Find_Speed_Duration_Direction(Stepper, GoTo);


    		            PRINTF("Stepper no: %d speed is %d \r\n",Stepper, speed);
    		            break;

    		case Stepper2:
    			Find_Speed_Duration_Direction(Stepper, GoTo);

    		    		PRINTF("Stepper no: %d speed is %d \r\n",Stepper, speed);
    		    		break;
    		case Stepper3:
    			Find_Speed_Duration_Direction(Stepper, GoTo);

    		      		PRINTF("Stepper no: %d speed is %d \r\n",Stepper, speed);
    		       		break;

    		}
    		Move(Duration1, Duration2, Duration3, Direction1,
    		    					Direction2, Direction3, speed1, speed2, speed3 );

        }
    }
};

Balancer_Thread balancer_thread("Balancer", BalancerPeriod * MILLISECONDS);
