#ifndef WSWATCHDOG_H
#define WSWATCHDOG_H
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <delta_kinematics.h>
#include <delta_math.hpp>
#include <eigen3/Eigen/Dense>
#include "delta_planner.hpp"

#define XY_LEN			100
#define Z_MAX			-253
#define Z_MIN			-390
#define Z_ROWS			138

class WSWatchdog
{

public:

  int Workspace[Z_ROWS][XY_LEN][2];
  WSWatchdog();
  virtual ~WSWatchdog();

  int readWorkSpace();
  bool checkPoint(const std::vector<float> point);
  int pointInPolygon(int nvert, float *vertx, float *verty, float testx, float testy);
  int giveBoundedPoint(float &x_prop, float &y_prop, float &z_prop);
  int calcPointOnPolygon(int nvert, float *vertx, float *verty, float testx, float testy,float &lpointx,float &lpointy);
  bool circleInWorkspace(const Eigen::Vector3d &circCenter, const Eigen::Vector3d &circAxis, const double &circRadius);

private:

};

#endif // WSWATCHDOG_H
