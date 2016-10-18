#include "deltaplanner.h"
#include <ros/ros.h>
#include <ros/package.h>

DeltaPlanner::DeltaPlanner():kinematics(32,80,295,100)
{

}
using namespace Eigen;
using namespace std;
void DeltaPlanner::getCubicAngle(float te, float t,const std::vector<float> & q_start, const std::vector<float> & q_end,std::vector<float> & q,std::vector<float> & dq){
//Berechnet Gelenkwinkel mittels kubischer Interpolation
//Da PTP Bewegung ist die Bewegung nicht geradlinig
  for(int i=0;i<3;i++){
    /*float a0 = q_start[i];
    float a2 = dq_end[i]/(2*te) - 3*(q_start[i]+ (dq_end[i] * te)/(2) - q_end[i])/(te*te);
    float a3 = (2*(q_start[i] + (dq_end[i]*te)/(2) - q_end[i]))/(te * te *te);

    q[i] = a0 + a2*t*t + a3*t*t*t;
    dq[i] = 2*a2*t + 3*a3*t*t;
    //float qdd = 2*a2 + 6*a3*t;*/
    calcCubicPoint(te,t,q_start[i],q_end[i],q[i],dq[i]);
  }
}
void DeltaPlanner::getCubicCartesian(float te, float t,const std::vector<float> & pos_start, const std::vector<float> & pos_end, std::vector<float> &x, std::vector<float> &dx, std::vector<float> &q,std::vector<float> &dq){
 //Berechnet Gelenkwinkel auf kubisch interpolierter PTP-Bahn im kartesischen (notwendig für echte Linearbewegungen)
  //float a0, a2, a3;
  //float ve=0;
  for (int i=0;i<3;i++){
  /*  a0 = pos_start[i];
    a2 = ve/(2*te) - 3*(pos_start[i]+ (ve * te)/(2) - pos_end[i])/(te*te);
    a3 = (2*(pos_start[i] + (ve*te)/(2) - pos_end[i]))/(te * te *te);
    x[i] = a0 + a2*t*t + a3*t*t*t;
    dx[i] = 2*a2*t + 3*a3*t*t;*/
    calcCubicPoint(te,t,pos_start[i],pos_end[i],x[i],dx[i]);
  }

  kinematics.delta_calcInverse(x,q);
  kinematics.delta_calcJointVel(x,dx,q,dq);

}


void DeltaPlanner::calcCircleFromThreePoints(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,const Eigen::Vector3d &p3,Eigen::Vector3d &circCenter,Eigen::Vector3d &circAxis,double &circRadius){
  //Berechne Kreis anhand dreier Punkte, die auf dem Kreis liegen
  // triangle "edges"
  const Vector3d t = p2-p1;
  const Vector3d u = p3-p1;
  const Vector3d v = p3-p2;

  // triangle normal
  const Vector3d w = t.cross(u);
  const double wsl = w.squaredNorm();

  // helpers
  const double iwsl2 = 1.0 / (2.0*wsl);
  const double tt = t.adjoint()*t;
  const double uu = u.adjoint()*u;
  const double vv = v.adjoint()*v;

  // result circle
  circCenter = p1 + (u*tt*(u.adjoint()*v) - t*uu*(t.adjoint()*v)) * iwsl2;
  circRadius = sqrt(tt * uu * vv * iwsl2*0.5);
  circAxis   = w/w.norm();

}

bool DeltaPlanner::circleInWorkspace(const Vector3d &circCenter, const Vector3d &circAxis, const double &circRadius)
{
  //Überprüfen ob ausgewählter Kreis komplett im Arbeitsraum liegt
  float theta_i = 0;
  vector<float> pos_i;
  Vector3d cao_helper(4.0+circAxis[0], 4.0+circAxis[0]+circAxis[1], 4.0+circAxis[0]+circAxis[1]+circAxis[2]);
  //Orthogonaler Vektor zu Kreisachse
  Vector3d cao = cao_helper.cross(circAxis);
  cao = cao/cao.norm();
  //Punkt auf Kreis
  Vector3d cp = circCenter + cao*circRadius;
  while(theta_i <= 2*M_PI){


    rotatePointAroundCircleAxis(cp ,pos_i,theta_i,circAxis);
    if(giveBoundedPoint(pos_i[0],pos_i[1],pos_i[2]) == 2) return false;
    theta_i += 2*M_PI/100;
  }
  return true;
}

void DeltaPlanner::calcCubicPoint(float te, float t_i, float p_start, float p_end, float &p_i, float &v_i){
  //Punkt auf kartesischem Polynom berechnen
   float a0, a2, a3;
   a0 = p_start;
   a2 = -3*(p_start- p_end)/(te*te);
   a3 = 2*(p_start - p_end)/(te * te *te);
   p_i = a0 + a2*t_i*t_i + a3*t_i*t_i*t_i;
   v_i = 2*a2*t_i + 3*a3*t_i*t_i;
}

void DeltaPlanner::rotatePointAroundCircleAxis(const Eigen::Vector3d &p,std::vector<float> &q, double theta,const Eigen::Vector3d &r)
{

   double costheta,sintheta;

   costheta = cos(theta);
   sintheta = sin(theta);

   q[0] = (costheta + (1 - costheta) * r[0] * r[0]) * p[0];
   q[0] += ((1 - costheta) * r[0] * r[1] - r[2] * sintheta) * p[1];
   q[0] += ((1 - costheta) * r[0] * r[2] + r[1] * sintheta) * p[2];

   q[1] = ((1 - costheta) * r[0] * r[1] + r[2] * sintheta) * p[0];
   q[1] += (costheta + (1 - costheta) * r[1] * r[1]) * p[1];
   q[1] += ((1 - costheta) * r[1] * r[2] - r[0] * sintheta) * p[2];

   q[2] = ((1 - costheta) * r[0] * r[2] - r[1] * sintheta) * p[0];
   q[2] += ((1 - costheta) * r[1] * r[2] + r[0] * sintheta) * p[1];
   q[2] += (costheta + (1 - costheta) * r[2] * r[2]) * p[2];

}
//##---WORKSPACE FUNCTIONS----##################################################
int DeltaPlanner::readWorkSpace() {
  //Read Workspace values, given in csv files. Each file describes a convex polygon in the xy-pane
  char buffer[32] = { 0 };
  char *record, *line;
  char path[100];
  char pathno[10];
  std::stringstream ss;
  std::stringstream pathstream;
  std::string s;

  for (int j = 0; j < Z_ROWS; j++) {
    int i = 0;

    std::string path_str = ros::package::getPath("delta_qtgui");
    if(path_str.compare("")==0){

      ss << "Path to delta_qtgui not found";

      s = ss.str();
      ROS_INFO(s.c_str());

      return -1;
    }
    pathstream.str("");
    pathstream << path_str<<"/resources/WorkspaceData/"<<(j - 390)<<".csv";

    snprintf(path,100,(pathstream.str()).c_str());
    FILE *fstream = fopen(path, "r");

    if (fstream == NULL)
    {
      ss.str("");
      ss << "Opening of file " << path << " failed";

      s =ss.str();
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
  //Check if Perpendicular point lies between pk1 and pk2, if not set point to pk1;
  float dp = (pk1_x - pk2_x)*(pk1_x - pk2_x) + (pk1_y - pk2_y)*(pk1_y - pk2_y);
  if (d2 > dp){
    lpointx=pk1_x;
    lpointy=pk2_y;
    return 0;
   }
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


  int countRows = 0;
  while (arrayEndFound == false) {
    if (Workspace[zInArray][countRows][0] == 9999) arrayEndFound = true;
    else countRows++;

  }

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
