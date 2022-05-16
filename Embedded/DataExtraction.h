/*
 * DataExtraction.h
 *
 *  Created on: Jul 2, 2018
 *      Author: info8
 */


/** Extracts data from strings with this format:
<FIELDNAME_1=VALUE_1><<FIELDNAME_2=VALUE_2>....<FIELDNAME_N=VALUE_N>(endofline)
*/

#pragma once

#include"rodos.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"
#include "..\topics.h"

char* strSkipUntilChar(char* source, char terminator);
char* strCpyUntilChar(char* dest, char* source, char terminator);
void  ExtractData(char *tmString,Telecommandmsg TelecommandDataref) ;

