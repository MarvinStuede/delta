#ifndef DELTAPLANNER_H
#define DELTAPLANNER_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>


#define XY_LEN			100
#define Z_MAX			-284
#define Z_MIN			-390
#define Z_ROWS			107

class DeltaPlanner
{
public:
   int Workspace[Z_ROWS][XY_LEN][2];
  DeltaPlanner();
  void getCubicAngle(float qs, float qz, float te, float t, float ve, float &q, float &qd);
  void getCubicCartesian(float te, float t, float* pos_start, float* pos__end, float *q[],float *qd[]);
  int readWorkSpace();
  int pointInPolygon(int nvert, float *vertx, float *verty, float testx, float testy);
  int giveBoundedPoint(float &x_prop, float &y_prop, float &z_prop);
  int calcPointOnPolygon(int nvert, float *vertx, float *verty, float testx, float testy,float &lpointx,float &lpointy);

private:

};

#endif // DELTAPLANNER_H
