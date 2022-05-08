/*****************************************************************
Telecommand.h

Original Created by: Atheel Redah @ University of Würzburg
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4 + Würzburg Uni Informatik 8 Discovery AddOn Board Version 2.0.
*****************************************************************/


#ifndef __Telecommand_h__
#define __Telecommand_h_


#define MaxLength  12

#define MotorID                                        'W'
#define StepperID                                      'B'

#define TelecommandStart                               '$'
#define TelecommandStop                                '#'
#define TelecommandAck                                 '&'

#define ThrusterID                                     'T'
#define ThreadPeriodID                                 'P'
#define ModeID                                         'M'
#define AHRSModeID                                     'A'
#define HWResetID                                      'H'

#define TelemetryID                                    't'
#define MotorSpeedID                                   's'
#define StepperSpeedID                                 'S'
#define VelocityID                                     'v'
#define PositionID                                     'o'
#define ControllerID                                   'C'
#define ControllerParameterID                          'p'
#define ControllerParameterGainID                      'g'

#define BearingsID                                     'b'
#define ThrustersID                                    'f'



#define PWMFreqID                                      'F'
#define PWMResID                                       'R'

/* Includes ------------------------------------------------------------------*/
#include "rodos.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"
#include "topics.h"

/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t Decode(uint8_t RxBuffer);
uint8_t Command(uint8_t TelecommandID);

#endif /* Telecommand_H_ */
