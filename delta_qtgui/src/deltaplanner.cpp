#include "deltaplanner.h"
#include <ros/ros.h>

DeltaPlanner::DeltaPlanner()
{

}
void DeltaPlanner::getCubicAngle(float qs, float qz, float te, float t, float ve, float &q, float &qd){

  if(qz<qs) ve = -ve;
  float a0 = qs;
  float a2 = ve/(2*te) - 3*(qs+ (ve * te)/(2) - qz)/(te*te);
  float a3 = (2*(qs + (ve*te)/(2) - qz))/(te * te *te);
  q = a0 + a2*t*t + a3*t*t*t;
  qd = 2*a2*t + 3*a3*t*t;
  float qdd = 2*a2 + 6*a3*t;

  //GEschwindigkeit ist immer mindestens a grad/s beim bremsen

 // if(qdd<0 && 0 < qd && qd < a) qd=a;
  //else if(qdd>0 && 0 > qd && qd > -a) qd=-a;

}
int DeltaPlanner::readWorkSpace() {
  //Read Workspace values, given in csv files. Each file describes a convex polygon in the xy-pane
  char buffer[32] = { 0 };
  char *record, *line;
  char path[100];
  char pathno[10];

  for (int j = 0; j < Z_ROWS; j++) {
    int i = 0;
    snprintf(path,100,"/home/marvin/catkin_ws/src/delta/delta_qtgui/resources/WorkspaceData/");
    snprintf(pathno, 10, "%d", (j - 390));
    strcat(path, pathno);
    strcat(path, ".csv");

    FILE *fstream = fopen(path, "r");

    if (fstream == NULL)
    {
      std::stringstream ss;
      ss << "Opening of file " << path << " failed";

      std::string s =ss.str();
      ROS_INFO(s.c_str());

      return -1;
    }
    while ((i < XY_LEN && fgets(buffer, sizeof(buffer), fstream)) != NULL)
    {
      if (sscanf(buffer, "%d\t%d", &Workspace[j][i][0], &Workspace[j][i][1]) == 2){
        i++;
      }
    }
    Workspace[j][i][0]=9999;
    fclose(fstream);
  }
  return 0;
}
int DeltaPlanner::pointInPolygon(int nvert, float *vertx, float *verty, float testx, float testy){
  //Check if testpoint (testx,testy) is in Polygon (vertx,verty) with nvert vertices
  int i, j, c = 0;
  for (i = 0, j = nvert - 1; i < nvert; j = i++) {
    vertx[i];
    verty[i];
    if (((verty[i]>testy) != (verty[j]>testy)) &&
      (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
      c = !c;
  }
  return c;
}
int DeltaPlanner::calcPointOnPolygon(int nvert, float *vertx, float *verty, float testx, float testy,float &lpointx,float &lpointy) {
  //Calculate point on Polygon boundary, which is the closest to the testpoint
  int i = 0;
  float pk1_x = 998, pk2_x = 999, pk1_y = 998, pk2_y = 999; //Two points of polygon closest to testpoint
  float d1 = 999999, d2 = 999999; //Distance to two closest points, initiliazed with absurdly high values
  float d = 0;
  //loop to calculate two closest points
  for (i = 0; i < nvert; i++) {
    d = (testx - vertx[i])*(testx - vertx[i]) + (testy - verty[i])*(testy - verty[i]);
    //check if point is closer that the two stored points
    if (d < d1 && d < d2) {

      d2 = d1;
      d1 = d;
      pk2_x = pk1_x;
      pk2_y = pk1_y;

      pk1_x = vertx[i];
      pk1_y = verty[i];
    }
    //check if point is closer than the second closest point
    else if (d < d2 && d >= d1) {
      if (pk1_x != vertx[i] || pk1_y != verty[i]) {
        d2 = d;
        pk2_x = vertx[i];
        pk2_y = verty[i];
      }
    }
  }

  //Normal vector for line between the two polygon points
  float n2 = -(pk2_x - pk1_x);
  float n1 = pk2_y - pk1_y;
  //Perpendicular point for testpoint on line between pk1 and pk2
  lpointx = testx - (((testx - pk1_x)*n1+ (testy - pk1_y)*n2) / (n1*n1 + n2*n2))*n1;
  lpointy = testy - (((testx - pk1_x)*n1 + (testy - pk1_y)*n2) / (n1*n1 + n2*n2))*n2;
  if (isnan(lpointx) || isnan(lpointy)){

    return -1;
  }
  else return 0;
}
int DeltaPlanner::giveBoundedPoint(float &x_prop, float &y_prop, float &z_prop) {
  //checks if proposed point is in Workspace, else calculates point on workspace boundary
  int i;
  bool arrayEndFound = false;
  //round z value and transform to array index
  int zInArray = (int)round(z_prop) - Z_MIN;
  std::stringstream ss;
  ss << zInArray;

  std::string s =ss.str();
  ROS_INFO(s.c_str());
  int countRows = 0;
  while (arrayEndFound == false) {
    if (Workspace[zInArray][countRows][0] == 9999) arrayEndFound = true;
    else countRows++;

  }
  countRows--;
  if (countRows > 0) {
    //Create helper array, which only contains the necessary xy-pane
    float *testArrayX;
    float *testArrayY;
    testArrayX = (float*)malloc(countRows * sizeof(float));
    testArrayY = (float*)malloc(countRows * sizeof(float));

    for (i = 0; i < countRows; i++) {
      testArrayX[i] = (float)Workspace[zInArray][i][0];
      testArrayY[i] = (float)Workspace[zInArray][i][1];
    }
    //Check if proposed xy-point is in boundary
    if (pointInPolygon(countRows, testArrayX, testArrayY, x_prop, y_prop)) {

      free(testArrayX);
      free(testArrayY);
      return 1; //1 indicates that point is inside workspace
    }
    else {
      float polyPointx, polyPointy;
      calcPointOnPolygon(countRows, testArrayX, testArrayY, x_prop, y_prop, polyPointx, polyPointy);
      x_prop = polyPointx;
      y_prop = polyPointy;
      free(testArrayX);
      free(testArrayY);
      return 2;//2 indicates that point would be outside of workspace and point on boundary was calculated
    }
  }
  else return -1;

}
