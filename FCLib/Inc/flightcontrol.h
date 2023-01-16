/*
 * flightcontrol.h
 *
 *  Created on: 2 Dec 2022
 *      Author: TTM
 */

#ifndef FCLIB_INC_FLIGHTCONTROL_H_
#define FCLIB_INC_FLIGHTCONTROL_H_

void FC_Setup();
int FC_Loop();
void FC_PassiveController();
void FC_CheckLandedStatus();
int GetSineSign(double value);
double filter(double angle);


#endif /* FCLIB_INC_FLIGHTCONTROL_H_ */
