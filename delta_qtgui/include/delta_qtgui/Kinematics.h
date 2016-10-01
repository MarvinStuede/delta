#ifndef Kinematics_h
#define Kinematics_h

#include "math.h"

class Kinematics
{
	public:
		Kinematics(float e_radius, float f_radius, float forearm, float biceps);
		int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0);
		int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3);
		int delta_calcParallelAngle(float x0, float y0, float z0, float theta1, float theta2, float theta3, float &phi1, float &phi2, float &phi3);
	private:
		int delta_calcAngleYZ(float x0, float y0, float z0, float &theta);
		int delta_calcAngleU1U2(float x0, float y0, float z0, float theta, float &phi, float rot);

     float sqrt3;
     float pi;
     float sin120;
     float cos120;
     float tan60;
     float sin30;
     float tan30;
		float e;
	    float f;
		float re;
		float rf;
};
#endif
