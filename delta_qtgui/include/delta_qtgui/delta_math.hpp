
#ifndef DELTA_MATH_H
#define DELTA_MATH_H

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <eigen3/Eigen/Dense>

namespace delta_math
{
  void rotatePointAroundCircleAxis(const Eigen::Vector3d &p,std::vector<float> &q, double theta,const Eigen::Vector3d &r);
  void calcCubicPoint(float te, float t_i, float p_start, float p_end, float &p_i, float &v_i);
  void calcCircleFromThreePoints(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,const Eigen::Vector3d &p3,Eigen::Vector3d &circCenter,Eigen::Vector3d &circAxis,double &circRadius);
}

#endif // DELTA_MATH_H
