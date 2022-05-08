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
#define SPI_STEPPER_SCK   GPIO_042		// PC10
#define SPI_STEPPER_MISO  GPIO_043		// PC11
#define SPI_STEPPER_MOSI  GPIO_044		// PC12
#define SPI_STEPPER_CS3   GPIO_027		// PB11

#define SPI_STEPPER       SPI_IDX3

#define L6470_CONFIG 	0x18
#define dSPIN_GET_PARAM 0x20
#define dSPIN_FORWARD	0x50
#define dSPIN_REVERSE	0x51

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
unsigned long Speed;
float Duration;
uint8_t Direction;
uint8_t Stepper;

CommBuffer<sTelecommandData> BalancerTelecommandDataBuffer;
Subscriber BalancerTelecommandDataSubscriber(TelecommandDataTopic, BalancerTelecommandDataBuffer);

HAL_GPIO stepper_cs(SPI_STEPPER_CS1);
HAL_GPIO stepper_cs1(SPI_STEPPER_CS2);
HAL_GPIO stepper_cs2(SPI_STEPPER_CS3);
HAL_SPI stepper_spi(SPI_STEPPER,SPI_STEPPER_SCK, SPI_STEPPER_MISO, SPI_STEPPER_MOSI);





struct sPID_Data
{
	 float Kp, Ki, Kd;

	 float P, I, D;

	 float e, e_1, Upid, Upid_1, Usat, Usat_1, Kr;

	 float Umax, Umin, T;

	 bool AntiWindup;
};

sPID_Data PID_StepperSpeed = {1, 0.5, 0,                               // float Kp, Ki, Kd;
					       0, 0, 0, 							     // float P, I, D;
					       0, 0, 0, 0, 0, 0, 1,                      // float e, e_1, Upid, Upid_1, Usat, Usat_1, Kr;
					       6, -6, (float)ModesPeriod/1000,     // float Umax, Umin, T;
                           1};                                       // bool AntiWindup;


void PID(sPID_Data* PID, float error)
{
	PID->e = (float) error;

	PID->P = PID->Kp*PID->e;

	if (PID->AntiWindup)
	{
		PID->I+= (PID->Ki*PID->e + PID->Kr * (PID->Usat_1 - PID->Upid_1)) * PID->T;
	}

	else {PID->I+= PID->Ki * PID->e * PID->T;}

	PID->D = PID->Kd * (PID->e - PID->e_1) / PID->T;

	PID->Upid = PID->P + PID->I + PID->D;

	if (PID->Upid >= PID->Umax)
	{
		PID->Usat = PID->Umax;
	}

	else if (PID->Upid <= PID->Umin)
	{
		PID->Usat = PID->Umin;
	}

	else
	{
		PID->Usat = PID->Upid;
	}

	PID->Upid_1 = PID->Upid;
	PID->Usat_1 = PID->Usat;
	PID->e_1 = PID->e;
}



void SteeringMode(int axis)
{
	switch(axis) {

	case 1:
	    StepperError = TelecommandDataReceiver.pitch - SensorDataReceiver.pitch;
	    PID(&PID_StepperSpeed, StepperError);
        
	    
	    set_stepper_speed(PID_StepperSpeed.Usat, 1);
	    break;

	case 2:
		StepperError = TelecommandDataReceiver.roll - SensorDataReceiver.roll;	
	    PID(&PID_Stepper, StepperError);
	    
	    set_stepper_speed(PID_StepperSpeed.Usat, 2);
	    break;

	case 3:
		StepperError = TelecommandDataReceiver.yaw - SensorDataReceiver.yaw;	
	    PID(&PID_Stepper, StepperError);
	    
	    set_stepper_speed(PID_StepperSpeed.Usat, 3);
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
	void set_stepper_speed(int32_t speed, int stepper_num){

		uint8_t data_send[4];
		uint16_t data_rcv;
		uint32_t abs_speed;

		switch (stepper_num){

		case 1:
		// direction definition parameter
		if(speed < 0){
			data_send[0] =  dSPIN_REVERSE;
			abs_speed = -speed*10000;
		}
		else{
			data_send[0] =  dSPIN_FORWARD;
			abs_speed = speed*10000;
		}

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
					abs_speed = -speed*10000;
				}
				else{
					data_send[0] =  dSPIN_FORWARD;
					abs_speed = speed*10000;
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
					abs_speed = -speed*10000;
				}
				else{
					data_send[0] =  dSPIN_FORWARD;
					abs_speed = speed*10000;
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




    	TIME_LOOP(0, periode){

    		BalancerTelecommandDataBuffer.getOnlyIfNewData(TelecommandDataReceiver);
    		Stepper = TelecommandDataReceiver.Stepper;


    		switch (Stepper) {

    		case Stepper1:
    		            BalancerTelecommandDataBuffer.getOnlyIfNewData(TelecommandDataReceiver);

    		            speed=TelecommandDataReceiver.StepperSpeed;

    		            set_stepper_speed((int32_t)speed, 1);

    		            PRINTF("Stepper no: %d speed is %d \r\n",Stepper, speed);
    		            break;

    		case Stepper2:
    		    		BalancerTelecommandDataBuffer.getOnlyIfNewData(TelecommandDataReceiver);

    		    		speed=TelecommandDataReceiver.StepperSpeed;

    		    		set_stepper_speed((int32_t)speed, 2);

    		    		PRINTF("Stepper no: %d speed is %d \r\n",Stepper, speed);
    		    		break;
    		case Stepper3:
    		    		BalancerTelecommandDataBuffer.getOnlyIfNewData(TelecommandDataReceiver);

    		       		speed=TelecommandDataReceiver.StepperSpeed;

     		    		set_stepper_speed((int32_t)speed, 3);

    		      		PRINTF("Stepper no: %d speed is %d \r\n",Stepper, speed);
    		       		break;

    		}

        }
    }
};

Balancer_Thread balancer_thread("Balancer", BalancerPeriod * MILLISECONDS); 