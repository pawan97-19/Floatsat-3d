/*
 *Balancer.cpp
 *
 *  Created on: Mar 11, 2022
 *      Author: Atheel Redah
 */

#include "rodos.h"
#include "topics.h"
#include <cmath>

#include "math.h"
#define PI = 3.142;

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

void set_stepper_speed(int stepperspeed, int stepper_num, float direction);
void Find_Speed_Duration_Direction(int StepperNumber, int GoTo[]);
void GOTO(float xAxis, float yAxis);


const int max_speed = 0x103FF; //maxiumum speed of the stepper motor according to data sheet = 1048575

uint32_t BalancerPeriod = 300; // Thread period in ms

enum Stepper
{
	Stepper1    = 1,
	Stepper2    = 2,

};

int MAXDISTANCE = 37; //37 mm
int distance1 = 37;
int distance2 = 0;

int speed1;
int speed2;

int Duration1;
int Duration2;

float Direction1;
float Direction2;

int Dist1;
int Dist2;

int Stepper;

CommBuffer<sTelecommandData> BalancerTelecommandDataBuffer;
Subscriber BalancerTelecommandDataSubscriber(TelecommandDataTopic, BalancerTelecommandDataBuffer);

HAL_GPIO stepper_cs(SPI_STEPPER_CS1);
HAL_GPIO stepper_cs1(SPI_STEPPER_CS2);
HAL_GPIO stepper_cs2(SPI_STEPPER_CS3);
HAL_SPI stepper_spi(SPI_STEPPER,SPI_STEPPER_SCK, SPI_STEPPER_MISO, SPI_STEPPER_MOSI);





void GOTO(float xAxis, float yAxis)
{
    float rot_matrix[4] = {0};
    float refAxis[2] = {1,1};
   rot_matrix[0] = cosd(xAxis);
    rot_matrix[1] = sind(xAxis);
    rot_matrix[2] = -sind(yAxis);
    rot_matrix[3] = cosd(yAxis);

    float GoTo[2] = {0};
    GoTo[0] = {rot_matrix[0]*refAxis[0] + rot_matrix[1]*refAxis[1]};
    GoTo[1] = {rot_matrix[2]*refAxis[0] + rot_matrix[3]*refAxis[1]};

}


void Find_Speed_Duration_Direction(int stepper, int GoTo[]) {

	switch (stepper) {
	case 1:
		Dist1 = abs(GoTo[0] - distance1);
		Direction1 = (GoTo[0] > distance1) ? dSPIN_FORWARD : dSPIN_REVERSE;
		distance1 = GoTo[0];
		speed1 = 35000;
		Duration1 = Dist1*0.3086;

		break;
	case 2:
		Dist2 = abs(GoTo[0] - distance2);
		Direction2 = (GoTo[0] > distance2) ? dSPIN_FORWARD : dSPIN_REVERSE;
		distance2 = GoTo[0];
		speed2 = 35000;
		Duration2 = Dist2* 0.3086;

		break;

	}

}

bool Steppper1 = false, Steppper2 = false;
void Move(float Duration1, float Duration2, float dir1,
		float dir2, int speed1, int speed2 ) {
	if (Duration1 > 0) {
		Steppper1 = true;
		set_stepper_speed(speed1, Stepper1, dir1);
	}
	if (Duration2 > 0) {
		Steppper2 = true;
		set_stepper_speed(speed2, Stepper2, dir2);
	}


	while (1) {
		AT(NOW() + 0.25*SECONDS);

		Duration1--;
		if (Duration1 <= 0) {
			set_stepper_speed(speed1, Stepper1, dir1);
			Steppper1 = false;
		}
		Duration2--;
		if (Duration2 <= 0) {
			set_stepper_speed(speed2, Stepper2, dir2);
			Steppper2 = false;
		}

		if (Steppper1 == false && Steppper2 == false ) {
			break;
		}
	}

}

void set_stepper_speed(int stepperspeed, int stepper_num, float direction)
	{

		uint8_t data_send[4];
		uint16_t data_rcv;
		uint32_t abs_speed ;

		switch (Stepper) {

		case Stepper1:
		// direction definition parameter

			data_send[0] =  direction;


						// direction definition parameter
						if(stepperspeed < 0){
							data_send[0] =  dSPIN_REVERSE;
							abs_speed = -stepperspeed;
						}
						else{
							data_send[0] =  dSPIN_FORWARD;
							abs_speed = stepperspeed;
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

		case Stepper2:

			// direction definition parameter

						data_send[0] =  direction;


				// direction definition parameter
				if(stepperspeed < 0){
					data_send[0] =  dSPIN_REVERSE;
					abs_speed = -stepperspeed;
				}
				else{
					data_send[0] =  dSPIN_FORWARD;
					abs_speed = stepperspeed;
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

/*	void Off(int stepper_num){
		set_stepper_speed(0, stepper_num, dSPIN_FORWARD );
	} */
    void run () {

		AT(NOW()+5*SECONDS);

		int speed=0;
		int GoTo[2];





    	TIME_LOOP(0, periode){

    		BalancerTelecommandDataBuffer.getOnlyIfNewData(TelecommandDataReceiver);
    		Stepper = TelecommandDataReceiver.Stepper;


    		switch (Stepper) {

    		case Stepper1:
    			GOTO(SensorData.roll, SensorData.pitch);
    			Find_Speed_Duration_Direction(Stepper, &GoTo[0]);


    		            PRINTF("Stepper no: %d speed is %d \r\n",Stepper, speed);
    		            break;

    		case Stepper2:
    			Find_Speed_Duration_Direction(Stepper, &GoTo[0]);

    		    		PRINTF("Stepper no: %d speed is %d \r\n",Stepper, speed);
    		    		break;


    		}
    		Move(Duration1, Duration2, Direction1,
    		    					Direction2, speed1, speed2);

        }
    }
};

Balancer_Thread balancer_thread("Balancer", BalancerPeriod * MILLISECONDS);
