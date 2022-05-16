/* * DataExtraction.cpp
 *
 *  Created on: Jul 2, 2018
 *      Author: info8
 */

#include"DataExtraction.h"

void ExtractData(char *tmString,Telecommandmsg TelecommandDataref) {


    char name[100];
    char value[100];
    char* next = tmString;
    char* subnext;
    char subvalue[100];
    int j=0;

    while(1) {
        next = strSkipUntilChar(next, '<');
        if(next == 0) break;
        next= strCpyUntilChar(name, next, '=');
        if(next == 0) break;
        next= strCpyUntilChar(value, next, '>');
        subnext = value;
        if(next == 0) break;

        if (strcmp(name,"WIFI") == 0 )
        {
              TelecommandDataref.WiFi = (uint8_t)(atof(value));

              // SystemMode = 0;
              // TelecommandDataTopic.publish(TelecommandData2);
        }

        if (strcmp(name,"CONTROL") == 0 )
       {
        	 subnext= strCpyUntilChar(subvalue, subnext, ',');
             TelecommandDataref.DCSpeed[0] = (int32_t)(atof(subvalue));
             memset(subvalue, 0, sizeof(subvalue));

        	 subnext= strCpyUntilChar(subvalue, subnext, ',');
             TelecommandDataref.DCSpeed[1] = (int32_t)(atof(subvalue));
             memset(subvalue, 0, sizeof(subvalue));

        	 subnext= strCpyUntilChar(subvalue, subnext, ',');
             TelecommandDataref.DCSpeed[2] = (int32_t)(atof(subvalue));
             memset(subvalue, 0, sizeof(subvalue));
             /* next control command */
        	 subnext= strCpyUntilChar(subvalue, subnext, ',');
             TelecommandDataref.STSpeed[0] = (int32_t)(atof(subvalue));
             memset(subvalue, 0, sizeof(subvalue));

        	 subnext= strCpyUntilChar(subvalue, subnext, ',');
             TelecommandDataref.STSpeed[1] = (int32_t)(atof(subvalue));
             TelecommandDataref.STSpeed[2] = (int32_t)(atof(subnext));

             TelecommandDataref.SystemMode = 11;
             // TelecommandDataTopic.publish(TelecommandData2);
       }

        if (strcmp(name,"DCCONTROL") == 0 )
       {
       	 subnext= strCpyUntilChar(subvalue, subnext, ',');
            TelecommandDataref.DCSpeed[0] = (int32_t)(atof(subvalue));
            memset(subvalue, 0, sizeof(subvalue));

       	 subnext= strCpyUntilChar(subvalue, subnext, ',');
            TelecommandDataref.DCSpeed[1] = (int32_t)(atof(subvalue));
            TelecommandDataref.DCSpeed[2] = (int32_t)(atof(subnext));

            TelecommandDataref.SystemMode = 9;
            // TelecommandDataTopic.publish(TelecommandData2);
       }

        if (strcmp(name,"STCONTROL") == 0 )
       {
       	 subnext= strCpyUntilChar(subvalue, subnext, ',');
            TelecommandDataref.STSpeed[0] = (int32_t)(atof(subvalue));
            memset(subvalue, 0, sizeof(subvalue));

       	 subnext= strCpyUntilChar(subvalue, subnext, ',');
            TelecommandDataref.STSpeed[1] = (int32_t)(atof(subvalue));
            TelecommandDataref.STSpeed[2] = (int32_t)(atof(subnext));

            TelecommandDataref.SystemMode = 10;
            // TelecommandDataTopic.publish(TelecommandData2);
       }

        if (strcmp(name,"ORIENTATION") == 0 )
       {
       	 subnext= strCpyUntilChar(subvalue, subnext, ',');
            TelecommandDataref.goal_orientation[0] = (double)(atof(subvalue));
            memset(subvalue, 0, sizeof(subvalue));

       	 subnext= strCpyUntilChar(subvalue, subnext, ',');
            TelecommandDataref.goal_orientation[1] = (double)(atof(subvalue));
            memset(subvalue, 0, sizeof(subvalue));

       	 subnext= strCpyUntilChar(subvalue, subnext, ',');
            TelecommandDataref.goal_orientation[2] = (double)(atof(subvalue));
            memset(subvalue, 0, sizeof(subvalue));
            /* next control command */
       	 subnext= strCpyUntilChar(subvalue, subnext, ',');
            TelecommandDataref.goal_orientation[3] = (double)(atof(subvalue));
            memset(subvalue, 0, sizeof(subvalue));

       	 subnext= strCpyUntilChar(subvalue, subnext, ',');
            TelecommandDataref.goal_orientation[4] = (double)(atof(subvalue));
            TelecommandDataref.goal_orientation[5] = (double)(atof(subnext));

            TelecommandDataref.SystemMode = 8;
            // TelecommandDataTopic.publish(TelecommandData2);
       }

        if (strcmp(name,"RELOCATION") == 0 )
       {
             TelecommandDataref.relocation = (int32_t)(atof(value));
             TelecommandDataref.SystemMode = 7;
             // TelecommandDataTopic.publish(TelecommandData2);
       }

        if (strcmp(name,"ESTIMATION") == 0 )
       {
             TelecommandDataref.estimation = (int32_t)(atof(value));
             TelecommandDataref.SystemMode = 6;
             // TelecommandDataTopic.publish(TelecommandData2);
       }

        if (strcmp(name,"CALIBRATION") == 0 )
       {
             TelecommandDataref.calibration = (int32_t)(atof(value));
             TelecommandDataref.SystemMode = 2;
             // TelecommandDataTopic.publish(TelecommandData2);
       }

        if (strcmp(name,"GYCALIBRATION") == 0 )
       {
             TelecommandDataref.calibration = (int32_t)(atof(value));
             TelecommandDataref.SystemMode = 3;
             // TelecommandDataTopic.publish(TelecommandData2);
       }

        if (strcmp(name,"ACCALIBRATION") == 0 )
       {
             TelecommandDataref.calibration = (int32_t)(atof(value));
             TelecommandDataref.SystemMode = 4;
             // TelecommandDataTopic.publish(TelecommandData2);
       }

        if (strcmp(name,"MGCALIBRATION") == 0 )
       {
             TelecommandDataref.calibration = (int32_t)(atof(value));
             TelecommandDataref.SystemMode = 5;
             // TelecommandDataTopic.publish(TelecommandData2);
       }

        if (strcmp(name,"VALIDATION") == 0 )
       {
             TelecommandDataref.valid = (int32_t)(atof(value));
             // TelecommandData2.SystemMode = 1;
             // TelecommandDataTopic.publish(TelecommandData2);
       }

        if (strcmp(name,"STOPALL") == 0 )
       {
             TelecommandDataref.stopall = (int32_t)(atof(value));
             TelecommandDataref.SystemMode = 12;
             // TelecommandDataTopic.publish(TelecommandData2);
       }
    }
}

char* strSkipUntilChar(char* source, char terminator) {
    if(source == 0) return 0;
    for(int i = 0; source[i] ; i++) {
        if(source[i] == terminator || source[i] == '\n') return source + i + 1;
    }
    return 0;
}

char* strCpyUntilChar(char* dest, char* source, char terminator) {
    if(source == 0) return 0;
    dest[0] = 0;
    for(int i = 0; source[i] ; i++) {
        if(source[i] == terminator || source[i] == '\n') return source + i + 1;
        dest[i] = source[i];
        dest[i+1] = 0;
    }
    return 0;
}
