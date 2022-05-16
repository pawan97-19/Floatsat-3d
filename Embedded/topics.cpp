/*****************************************************************
topics.cpp

Original Created by: Atheel Redah @ University of Würzburg
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4 + Würzburg Uni Informatik 8 Discovery AddOn Board Version 2.0.
 *****************************************************************/

#include "rodos.h"
#include "topics.h"

long int Rpm_max[3] = {8000,8000,8000};
long int Rpm_min[3] = {-8000,-8000,-8000 };

void matrix_Const_mult(double constant, double* mat,int matlenght, double* result)
{
	for (int i = 0; i < matlenght;i++) {
		*(result + i) = constant * *(mat + i);
	}

}
void matrix_add(double* mat1, double* mat2, int matlenght, double* result)
{
	for (int i = 0; i < matlenght;i++) {
		*(result + i) = *(mat1 + i) + *(mat2 + i);
	}
}
void normalize_vect(double* vect)
{
	double normVect = normvect(vect);

	*(vect + 0) = (normVect == 0) ? 0 : (*(vect + 0)) / normVect;
	*(vect + 1) = (normVect == 0) ? 0 : (*(vect + 1)) / normVect;
	*(vect + 2) = (normVect == 0) ? 0 : (*(vect + 2)) / normVect;

}

void cross_prod(double *a, double *b, double *cv)
{
	*(cv + 0) = *(a + 1) * *(b + 2) - *(a + 2) * *(b + 1);
	*(cv + 1) = *(a + 2) * *(b + 0) - *(a + 0) * *(b + 2);
	*(cv + 2) = *(a + 0) * *(b + 1) - *(a + 1) * *(b + 0);
}

double dot_prod(double* a, double* b)
{
	return  *(a + 0) * *(b + 0)  + *(a + 1) * *(b + 1) + *(a + 2) * *(b + 2);
}

double normvect(double* vect)
{
	return sqrt(pow(*(vect + 0), 2) + pow(*(vect + 1), 2) + pow(*(vect + 2), 2));
}

void AxAng2Quat(double* axis, double angle,double* quat)
{
	*(quat + 0) = *(axis + 0) * sin(angle / 2);
	*(quat + 1) = *(axis + 1) * sin(angle / 2);
	*(quat + 2) = *(axis + 2) * sin(angle / 2);
	*(quat + 3) = cos(angle / 2);
}

void vect2Quat(double* Ar, double* quat)
{
	*(quat + 0) = (normvect(Ar) == 0) ? 0 : *(Ar + 0) / normvect(Ar);
	*(quat + 1) = (normvect(Ar) == 0) ? 0 : *(Ar + 1) / normvect(Ar);
	*(quat + 2) = (normvect(Ar) == 0) ? 0 : *(Ar + 2) / normvect(Ar);
	*(quat + 3) = 0;

}

void hamilton_prod(double* quaternion2, double* quaternion1, double* quaternion12)
{
	double quatangle1 = *(quaternion1 + 3);
	double quatangle2 = *(quaternion2 + 3);
	double subQuaternion1[3] = { *(quaternion1 + 0),*(quaternion1 + 1) ,*(quaternion1 + 2) };
	double subQuaternion2[3] = { *(quaternion2 + 0),*(quaternion2 + 1) ,*(quaternion2 + 2) };

	*(quaternion12 + 3) = quatangle1 * quatangle2 - dot_prod(subQuaternion1, subQuaternion2);

	double quat_temp1[3] = {0,0,0};
	cross_prod(subQuaternion1, subQuaternion2,quat_temp1);

	double quat_temp2[3] = { 0,0,0 };
	double quat_temp3[3] = { 0,0,0 };
	double quat_temp4[3] = { 0,0,0 };
	double quat_temp5[3] = { 0,0,0 };

	matrix_Const_mult(quatangle1, subQuaternion2, 3, quat_temp2);
	matrix_Const_mult(quatangle2, subQuaternion1, 3, quat_temp3);
	matrix_add(quat_temp2, quat_temp3, 3, quat_temp4);

	matrix_add(quat_temp4, quat_temp1, 3, quaternion12);
}

void quat_conj(double* Q, double* result)
{
	*(result + 0) = -*(Q + 0);
	*(result + 1) = -*(Q + 1);
	*(result + 2) = -*(Q + 2);
	*(result + 3) = *(Q + 3);
}

void quat2eulang(double* quaternion12, double* eulerang)
{
	*(eulerang + 0) = atan2(2 * (*(quaternion12 + 0) * *(quaternion12 + 3) + *(quaternion12 + 2) * *(quaternion12 + 1)),1-2*(pow(*(quaternion12 + 0),2) + pow(*(quaternion12 + 1),2)));
	*(eulerang + 1) = asin(2*(*(quaternion12 + 1) * *(quaternion12 + 3) - *(quaternion12 + 2) * *(quaternion12 + 0)));
	*(eulerang + 2) = atan2(2 * (*(quaternion12 + 3) * *(quaternion12 + 2) + *(quaternion12 + 0) * *(quaternion12 + 1)), 1 - 2 * (pow(*(quaternion12 + 1), 2) + pow(*(quaternion12 + 2), 2)));
}

void eul2RotMat(double* eulAng, double* rotmat)
{
	double rx[9] = { 1,0,0,0,cos(*(eulAng + 0)),-sin(*(eulAng + 0)),0,sin(*(eulAng + 0)),cos(*(eulAng + 0))};
	double ry[9] = { cos(*(eulAng + 1)),0,sin(*(eulAng + 1)),0,1,0,-sin(*(eulAng + 1)),0,cos(*(eulAng + 1)) };
	double rz[9] = { cos(*(eulAng + 2)),-sin(*(eulAng + 2)),0,sin(*(eulAng + 2)),cos(*(eulAng + 2)),0,0,0,1 };

	double rx_a[3][3] = { {1,0,0},{0,cos(*(eulAng + 0)),-sin(*(eulAng + 0))},{0,sin(*(eulAng + 0)),cos(*(eulAng + 0))} };
	double rx_at[3][3] = { {1,0,0},{0,cos(*(eulAng + 0)),sin(*(eulAng + 0))},{0,-sin(*(eulAng + 0)),cos(*(eulAng + 0))} };

	double ry_a[3][3] = { {cos(*(eulAng + 1)),0,sin(*(eulAng + 1))},{0,1,0},{-sin(*(eulAng + 1)),0,cos(*(eulAng + 1))} };
	double ry_at[3][3] = { {cos(*(eulAng + 1)),0,-sin(*(eulAng + 1))},{0,1,0},{sin(*(eulAng + 1)),0,cos(*(eulAng + 1))} };

	double rz_a[3][3] = { {cos(*(eulAng + 2)),-sin(*(eulAng + 2)),0},{sin(*(eulAng + 2)),cos(*(eulAng + 2)),0},{0,0,1} };

	double temp_mat_t[3][3] = { {dot_prod(ry_a[0],rx_at[0]),dot_prod(ry_a[1],rx_at[0]),dot_prod(ry_a[2],rx_at[0])},
			{dot_prod(ry_a[0],rx_at[1]),dot_prod(ry_a[1],rx_at[1]),dot_prod(ry_a[2],rx_at[1])},
			{dot_prod(ry_a[0],rx_at[2]),dot_prod(ry_a[1],rx_at[2]),dot_prod(ry_a[2],rx_at[2])} };


	double temp_mat_t2[3][3] = {{dot_prod(rz_a[0],temp_mat_t[0]),dot_prod(rz_a[1],temp_mat_t[0]),dot_prod(rz_a[2],temp_mat_t[0])},
			{dot_prod(rz_a[0],temp_mat_t[1]),dot_prod(rz_a[1],temp_mat_t[1]),dot_prod(rz_a[2],temp_mat_t[1])},
			{dot_prod(rz_a[0],temp_mat_t[2]),dot_prod(rz_a[1],temp_mat_t[2]),dot_prod(rz_a[2],temp_mat_t[2])} };

	*(rotmat + 0) = temp_mat_t2[0][0];
	*(rotmat + 1) = temp_mat_t2[1][0];
	*(rotmat + 2) = temp_mat_t2[2][0];

	*(rotmat + 3) = temp_mat_t2[0][1];
	*(rotmat + 4) = temp_mat_t2[1][1];
	*(rotmat + 5) = temp_mat_t2[2][1];

	*(rotmat + 6) = temp_mat_t2[0][2];
	*(rotmat + 7) = temp_mat_t2[1][2];
	*(rotmat + 8) = temp_mat_t2[2][2];


	//  RotMat =  R_z*R_y*R_x;
}

void attitudeplanning_Run(double* a, double* b, double* eulang)
{
	double zg[3] = {*(a + 0),*(a + 1),*(a + 2) };
	double xg[3] = { *(a + 3),*(a + 4),*(a + 5) };

	double za[3] = { *(b + 0),*(b + 1),*(b + 2) };
	double xa[3] = { *(b + 3),*(b + 4),*(b + 5) };

	normalize_vect(zg);
	normalize_vect(xg);

	normalize_vect(za);
	normalize_vect(xa);

	/* first Rotation sequence */

	double temp_vec[3] = {0,0,0};
	cross_prod(za, zg, temp_vec);

	double angl = atan2(normvect(temp_vec), dot_prod(za,zg));

	double orth_vec[3] = { 0,0,0 };

	orth_vec[0] = za[1]*(-4) + za[2];
	orth_vec[1] = za[0] * (4);
	orth_vec[2] = za[0] * (-1);

	double axis_vec[3] = { 0,0,0 };

	double normVect = normvect(temp_vec);

	*(axis_vec + 0) = (normVect == 0) ? *(orth_vec + 0) : (*(temp_vec + 0));
	*(axis_vec + 1) = (normVect == 0) ? *(orth_vec + 1) : (*(temp_vec + 1));
	*(axis_vec + 2) = (normVect == 0) ? *(orth_vec + 2) : (*(temp_vec + 2));

	normalize_vect(axis_vec);

	double quat1[4] = {0,0,0,0};

	AxAng2Quat(axis_vec, angl, quat1);

	/* new actual coord of the X vector */

	double quata[4] = { 0,0,0,0 };
	double quaternion1_[4] = { 0,0,0,0 };
	double quaternion12[4] = { 0,0,0,0 };

	vect2Quat(xa, quata);

	hamilton_prod(quata, quat1, quaternion1_);

	double quatconj[4] = {0,0,0};

	quat_conj(quat1, quatconj);

	hamilton_prod(quatconj, quaternion1_, quaternion12);

	double xac[3] = { 0,0,0};

	*(xac + 0) = *(quaternion12 + 0);
	*(xac + 1) = *(quaternion12 + 1);
	*(xac + 2) = *(quaternion12 + 2);

	/* Second Rotation sequence */

	double temp_vec2[3] = { 0,0,0 };
	cross_prod(xac, xg, temp_vec2);

	double angl2 = atan2(normvect(temp_vec2), dot_prod(xac, xg));

	double axis_vec2[3] = { 0,0,0 };

	double normVect2 = normvect(temp_vec2);

	*(axis_vec2 + 0) = (normVect2 == 0) ? *(zg + 0) : (*(temp_vec2 + 0));
	*(axis_vec2 + 1) = (normVect2 == 0) ? *(zg + 1) : (*(temp_vec2 + 1));
	*(axis_vec2 + 2) = (normVect2 == 0) ? *(zg + 2) : (*(temp_vec2 + 2));

	normalize_vect(axis_vec2);

	double quat2[4] = {0,0,0,0};

	AxAng2Quat(axis_vec2, angl2, quat2);

	/* Euler Angles computation */

	double Quaternion12[4] = { 0,0,0,0 };

	hamilton_prod(quat1, quat2, Quaternion12);

	quat2eulang(Quaternion12, eulang);

	*(eulang + 0) = fmod(*(eulang + 0), 2 * PI);
	*(eulang + 1) = fmod(*(eulang + 1), 2 * PI);
	*(eulang + 2) = fmod(*(eulang + 2), 2 * PI);

}

void motorcontrol_Run(double* posediff, double* dws, float delta_t, double* Is, double* Irw, long int* Rpm)
{
	/*
	- posediff: position difference in Rad.
	- w: satellite velocity in rad/s.
	- delta_t: thread period in s
	- Is
	- Irw
	- Rpm
	 */

	// posediff represent <<e>> for the PID controller and delta_t is the thread period.
	double posevar[3] = {0.0,0.0,0.0}; // represent the position variation as derivativ of <<e>> for the PID controller
	matrix_Const_mult(pow((double)delta_t,2), dws, 3, posevar);

	// go inside the PID controller and actualize  the controller response <<u(t)>>
	double u[3] = { 0.0,0.0,0.0 }; // control response as the next velocity
	PID_controller(posediff, posevar, 1, 0.5, 0.9, u);

	// compute the new satellite velocity
	double ws[3] = { 0.0,0.0,0.0 }; // satellite velocity
	matrix_Const_mult(delta_t, dws, 3, ws);
	matrix_add(u, ws, 3, ws);

	// compute the Flywheels velocity using the simplified equation of motion dwrw = (Is/Irw)*dws.
	double w[3] = { 0.0,0.0,0.0 }; // flywheel velocity
	*(w + 0) = - (*(Is + 0) / *(Irw + 0)) * *(ws + 0);
	*(w + 1) = - (*(Is + 1) / *(Irw + 1)) * *(ws + 1);
	*(w + 2) =   (*(Is + 2) / *(Irw + 2)) * *(ws + 2);

	// convert the flywheel velocity from rad/s to Rpm by multiplying with 60/2pi
	*(Rpm + 0) = (int)floor(*(w + 0) * (60 / (2 * PI)));
	*(Rpm + 1) = (int)floor(*(w + 1) * (60 / (2 * PI)));
	*(Rpm + 2) = (int)floor(*(w + 2) * (60 / (2 * PI)));

	// check for saturation of the comtrol response
	*(Rpm + 0) = (*(Rpm + 0) > *(Rpm_max + 0)) ? *(Rpm_max + 0) : (*(Rpm + 0) < *(Rpm_min + 0)) ? *(Rpm_min + 0) : *(Rpm + 0);
	*(Rpm + 1) = (*(Rpm + 1) > *(Rpm_max + 1)) ? *(Rpm_max + 1) : (*(Rpm + 1) < *(Rpm_min + 1)) ? *(Rpm_min + 1) : *(Rpm + 1);
	*(Rpm + 2) = (*(Rpm + 2) > *(Rpm_max + 2)) ? *(Rpm_max + 2) : (*(Rpm + 2) < *(Rpm_min + 2)) ? *(Rpm_min + 2) : *(Rpm + 2);

}

void PID_controller(double* e, double* de, double Kp, double Ki, double Kd, double* ut)
{
	*(ut + 0) = Kp * *(e + 0) + Kd * *(de + 0) + Ki * (2 * *(e + 0) - *(de + 0));
	*(ut + 1) = Kp * *(e + 1) + Kd * *(de + 1) + Ki * (2 * *(e + 1) - *(de + 1));
	*(ut + 2) = Kp * *(e + 2) + Kd * *(de + 2) + Ki * (2 * *(e + 2) - *(de + 2));
}

Topic<Telecommandmsg> TelecommandDataTopic(-1,"Telecommand Data");
Topic<APmsg> AttitudeplanningTopic(-1,"attitude planing Data");
Topic<ImuDatamsg> ImuTopic(-1,"IMU Data");
Topic<AC2Telemetrie> AC2TelemetrieTopic(-1,"AC2Telemetrie Data");
Topic<CM2Telemetrie> CM2TelemetrieTopic(-1,"CM2Telemetrie Data");
