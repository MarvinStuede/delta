#ifndef Kinematics_h
#define Kinematics_h

#include "math.h"
#include <vector>
#include <eigen3/Eigen/Dense>
class Kinematics
{

	public:

      Kinematics(float e_radius, float f_radius, float forearm, float biceps);
      int delta_calcForward(const std::vector<float> &theta,  std::vector<float> &x);
      int delta_calcInverse(const std::vector<float> &x,  std::vector<float> &theta);
      int delta_calcParallelAngle(const std::vector<float> &x,const std::vector<float> &theta, std::vector<float> &psi,std::vector<float> &phi);
      void delta_DirJacMatrix(const std::vector<float> &phi, const std::vector<float> &psi, const std::vector<float> &theta, Eigen::Matrix3f& OUTPUT);
      void delta_InvJacMatrix(const std::vector<float> &phi, const std::vector<float> &psi,Eigen::Matrix3f& OUTPUT);
      int delta_calcJointVel(const std::vector<float> &x,std::vector<float> &dx,std::vector<float> &theta, std::vector<float> &dtheta);
      void deg2rad(std::vector<float> &x);
      void rad2deg(std::vector<float> &x);

	private:
		int delta_calcAngleYZ(float x0, float y0, float z0, float &theta);
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
