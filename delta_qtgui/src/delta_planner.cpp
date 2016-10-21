#include "delta_planner.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <QApplication>

DeltaPlanner::DeltaPlanner(delta_qtgui::QNode &qnode)
  :kinematics(32,80,295,100)
{
    //Pointer auf Node zur Kommunikation mit dem Delta
  qn = &qnode;
  movFlag = false;

}

DeltaPlanner::~DeltaPlanner()
{
  Q_EMIT finished();
}
using namespace delta_qtgui;
using namespace Eigen;
using namespace std;
//##########  DRAW FUNCTIONS     #################################
int DeltaPlanner::drawYAML(std::string filename,std::vector<float> &start_pos,float vmax){
//Fahre Punkte ab, die in YAML-Datei gegeben sind
  vector<float> relPos(3,0);
  delta_posereader::PoseMap poses=delta_posereader::read(filename);


  if (movePTP(start_pos)){

    delta_posereader::PoseMap::iterator it=poses.begin();

    do{
      //Relative Position nach Reihenfolge in YAML setzen
      relPos[0] = it->second.x;
      relPos[1] = it->second.y;
      relPos[2] = it->second.z;
      if(it==poses.end()) break;
      ++it;
    }
    while(movePTP(relPos,vmax,0.005,CUBICCART,RELATIVE));

   }
}

//##########  MOTION FUNCTIONS   ##################################
void DeltaPlanner::setFlag(const bool en){
   movFlag = en;
}
int DeltaPlanner::moveCircular(vector<vector<float> > &circlePoints, float angle, float vmax, float stepSize){
    //Fahre Kreis bestehend aus drei Punkten
    vector<float> q_start(3,0);
    vector<float> pos_start(3,0);
    vector<float> dq(3,0);
    vector<float> q(3,0);
    Vector3d p1(circlePoints[0][0],circlePoints[0][1],circlePoints[0][2]);
    Vector3d p2(circlePoints[1][0],circlePoints[1][1],circlePoints[1][2]);
    Vector3d p3(circlePoints[2][0],circlePoints[2][1],circlePoints[2][2]);
    //Kreismittelpunkt, -achse und -radius definieren
    Vector3d circCenter;
    Vector3d circAxis;
    double circRadius;
    //Kreis berechnen
    calcCircleFromThreePoints(p1,p2,p3,circCenter,circAxis,circRadius);

    //Punkt auf Kreis in aktueller Iteration
    vector<float> pos_i(3,0);

    //Winkel der gefahren werden soll
    float theta_end = angle * M_PI/180;
    float theta_i = 0;
    float dtheta_i = 0;

    //Gesamtdauer für Kreisfahrt
    float te = fabs(1.5*theta_end*circRadius/vmax);

    qn->getDeltaAngles("GETANGLES", q_start);
    kinematics.delta_calcForward(q_start,pos_start);

    ros::Time endTime = ros::Time::now() + ros::Duration(te);
    ros::Time startTime = ros::Time::now();
    ros::Rate rate(1/stepSize);
    vector<float> xi(3,0);
    movFlag = true;

    while(ros::ok() && ros::Time::now()<= endTime && movFlag){
      ros::Duration tr = ros::Time::now() - startTime;
      double t = tr.toSec();
      //Gebe Punkt auf Kreis an bestimmtem Winkel zurück (Im Kreiskoord.-system)
      calcCubicPoint(te,t,0,theta_end,theta_i,dtheta_i);

      rotatePointAroundCircleAxis(p1 - circCenter ,pos_i,theta_i,circAxis);
      //In Taskspace transformieren
      for(int i=0;i<3;i++) xi[i]= pos_i[i] + circCenter[i];

      qn->sendDeltaCart(xi);
      dq[0]=40;
      dq[1]=40;
      dq[2]=40;

      kinematics.delta_calcInverse(xi,q);
      kinematics.rad2deg(q);
      qn->sendDeltaAngle(q,dq);
      ros::spinOnce();
      qApp->processEvents();

      rate.sleep();
    }
    movFlag = false;
}
int DeltaPlanner::movePTP(std::string posname, float vmax, float stepSize,const InterpolationMode &imode,const MotionMode &mmode){
     //Überladene Funktion um Punkt nach namen anzufahren
     delta_posereader::PoseMap poses=delta_posereader::read();
     vector<float> pos_end(3,0);

     if(poses.find(posname) != poses.end()){
       pos_end[0]=poses[posname].x;
       pos_end[1]=poses[posname].y;
       pos_end[2]=poses[posname].z;
       return movePTP_(pos_end, vmax,stepSize,imode,mmode);
     }
     return -1;
}
int DeltaPlanner::movePTP(vector<float> &pos_end, float vmax, float stepSize,const InterpolationMode &imode,const MotionMode &mmode){

    return movePTP_(pos_end, vmax,stepSize,imode,mmode);
}
int DeltaPlanner::movePTP_(vector<float> &pos_end, float vmax, float stepSize,const InterpolationMode &imode,const MotionMode &mmode){

    //Goal/Start
    vector<float> q_end(3,0);
    vector<float> q_start(3,0);
    vector<float> pos_start(3,0);

    //Current Point/Velocity
    vector<float> q_i(3,0);
    vector<float> dq_i(3,40);
    vector<float> x_i(3,0);
    vector<float> dx_i(3,0);

   float te = 0;

    //Aktuelle Winkel in Jointkoordinaten auslesen und EE Pos. berechnen
   qn->getDeltaAngles("GETANGLES", q_start);
   kinematics.delta_calcForward(q_start,pos_start);

   //Bei Relativbewegung Ziel in Absolutposition ausrechnen
   if(mmode == RELATIVE){
    for(int i=0;i<3;i++) pos_end[i] += pos_start[i];
   }

   //Endpunkt in Jointkoordinaten berechnen
   kinematics.delta_calcInverse(pos_end,q_end);
   kinematics.rad2deg(q_end);

   //Meldung machen
   std::stringstream ss;
   ss<<"Moving to x: "<<pos_end[0]<<" y: "<<pos_end[1]<<" z: "<<pos_end[2];
   qn->log(QNode::Info,ss.str());

   switch(imode){
   //--------------  DIREKT --------------------
   case NONE:{
     //Zielposition direkt an Roboter senden (Motoren fahren zu Zielwinkel mit selber Geschwindigkeit)
     qn->sendDeltaCart(pos_end);
     qn->sendDeltaAngle(q_end,dq_i);
     break;
   }
   //--------------  LINEAR --------------------
   case LINEAR:{
    //Synchronisierte Linearbewegung mit konstanter Geschwindigkeit
     vector<float> dist(3,0);
     for(int i = 0;i<3;i++){
         dist[i]=fabs(q_end[i] - q_start[i]);

         if (dist[i] / vmax > te) te = dist[i]/vmax;
     }

     dq_i[0]=dist[0]/te;
     dq_i[1]=dist[1]/te;
     dq_i[2]=dist[2]/te;
     qn->sendDeltaCart(pos_end);
     qn->sendDeltaAngle(q_end,dq_i);
     break;

   }
   //--------------  KUBISCH --------------------
   case CUBICCART:{

     //Bei kubischer Bewegung Dauer anhand max. Geschw. berechnen
     float tj = 0;
     for(int j=0;j<3;j++){
       //Dauer für Bewegung anhand Max. Geschwindigkeit
       //vmax erreicht bei te/2
         tj=fabs(1.5*(q_end[j] - q_start[j]) / vmax);
         if (tj > te) te = tj;
      }

     ros::Time endTime = ros::Time::now() + ros::Duration(te);
     ros::Time startTime = ros::Time::now();
     ros::Rate rate(1/stepSize);

     //Schleife für Point Kommandierung
     //Aus der GUI kann Flag zum stoppen des Roboters gesetzt werden
     movFlag = true;
     while(ros::ok() && ros::Time::now()<= endTime && movFlag){
       ros::Duration tr = ros::Time::now() - startTime;
       double t = tr.toSec();

       for (int i=0;i<3;i++){
         calcCubicPoint(te,t,pos_start[i],pos_end[i],x_i[i],dx_i[i]);
       }

       kinematics.delta_calcInverse(x_i,q_i);
       kinematics.delta_calcJointVel(x_i,dx_i,q_i,dq_i);
       kinematics.rad2deg(q_i);

       //Konstante Geschwindigkeit setze (Problem.. :( Verzögerung der Umsetzung)
       dq_i[0]=40;
       dq_i[1]=40;
       dq_i[2]=40;
       qn->sendDeltaCart(x_i);
       qn->sendDeltaAngle(q_i,dq_i);
       ros::spinOnce();

       //Während Schleife auf Eingaben hören
        qApp->processEvents();
       //Ggf. warten damit Frequenz nicht überschritten wird
       rate.sleep();

     }
     //0 zurückgeben wenn Bewegung abgebrochen
     if(!movFlag) return 0;
     movFlag = false;
     break;
   }
   }

    return 1;
}

//##########  CUBIC FUNCTIONS   ##################################
void DeltaPlanner::getCubicAngle(float te, float t,const std::vector<float> & q_start, const std::vector<float> & q_end,std::vector<float> & q,std::vector<float> & dq){
//Berechnet Gelenkwinkel mittels kubischer Interpolation
//Da PTP Bewegung ist die Bewegung nicht geradlinig
  for(int i=0;i<3;i++){
    calcCubicPoint(te,t,q_start[i],q_end[i],q[i],dq[i]);
  }
}
void DeltaPlanner::getCubicCartesian(float te, float t,const std::vector<float> & pos_start, const std::vector<float> & pos_end, std::vector<float> &x, std::vector<float> &dx, std::vector<float> &q,std::vector<float> &dq){
 //Berechnet Gelenkwinkel auf kubisch interpolierter PTP-Bahn im kartesischen (notwendig für echte Linearbewegungen)

  for (int i=0;i<3;i++){
    calcCubicPoint(te,t,pos_start[i],pos_end[i],x[i],dx[i]);
  }

  kinematics.delta_calcInverse(x,q);
  kinematics.delta_calcJointVel(x,dx,q,dq);

}
void DeltaPlanner::calcCubicPoint(float te, float t_i, float p_start, float p_end, float &p_i, float &v_i){
  //Punkt auf kubischem Polynom berechnen
   float a0, a2, a3;
   a0 = p_start;
   a2 = -3*(p_start- p_end)/(te*te);
   a3 = 2*(p_start - p_end)/(te * te *te);
   p_i = a0 + a2*t_i*t_i + a3*t_i*t_i*t_i;
   v_i = 2*a2*t_i + 3*a3*t_i*t_i;
}

//##########  CIRCLE FUNCTIONS   ##################################
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
void DeltaPlanner::rotatePointAroundCircleAxis(const Eigen::Vector3d &p,std::vector<float> &q, double theta,const Eigen::Vector3d &r)
{
  //Punkt p um Winkel theta um Achse r rotieren, ergibt q

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
bool DeltaPlanner::circleInWorkspace(const Vector3d &circCenter, const Vector3d &circAxis, const double &circRadius)
{
  //Überprüfen ob ausgewählter Kreis komplett im Arbeitsraum liegt
  float theta_i = 0;
  vector<float> pos_i(3,0);
  Vector3d cao_helper(4.0+circAxis[0], 4.0+circAxis[0]+circAxis[1], 4.0+circAxis[0]+circAxis[1]+circAxis[2]);
  //Orthogonaler Vektor zu Kreisachse
  Vector3d cao = cao_helper.cross(circAxis);
  cao = cao/cao.norm();
  //Punkt auf Kreis
  Vector3d cp = circCenter + cao*circRadius;

 while(theta_i <= 2*M_PI){


    rotatePointAroundCircleAxis(cp ,pos_i,theta_i,circAxis);
    if(giveBoundedPoint(pos_i[0],pos_i[1],pos_i[2]) == 2) return false;
    //100 verschiedene Winkel überprüfen
    theta_i += 2*M_PI/100;
  }
  return true;
}
//##   WORKSPACE FUNCTIONS      ##################################################
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
