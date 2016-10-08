#ifndef DELTAPLANNER_H
#define DELTAPLANNER_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <Kinematics.h>

#define XY_LEN			100
#define Z_MAX			-284
#define Z_MIN			-390
#define Z_ROWS			107

class DeltaPlanner
{
public:
   int Workspace[Z_ROWS][XY_LEN][2];
  DeltaPlanner();
  void getCubicAngle(float te, float t,const std::vector<float> & q_start, const std::vector<float> & q_end,std::vector<float> & dq_end,std::vector<float> & q,std::vector<float> & dq);
  void getCubicCartesian(float te, float t,const std::vector<float> & pos_start, const std::vector<float> & pos_end,std::vector<float> &x, std::vector<float> &dx, std::vector<float> &q,std::vector<float> &qd);
  int readWorkSpace();
  int pointInPolygon(int nvert, float *vertx, float *verty, float testx, float testy);
  int giveBoundedPoint(float &x_prop, float &y_prop, float &z_prop);
  int calcPointOnPolygon(int nvert, float *vertx, float *verty, float testx, float testy,float &lpointx,float &lpointy);

private:
  Kinematics kinematics;

};

#endif // DELTAPLANNER_H
