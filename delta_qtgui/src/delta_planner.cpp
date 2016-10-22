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
int DeltaPlanner::drawYAML(std::string filename,std::vector<float> &start_pos,float vmax,const MotionMode &mmode){
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
    while(movePTP(relPos,vmax,0.005,CUBICCART,mmode));

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
    delta_math::calcCircleFromThreePoints(p1,p2,p3,circCenter,circAxis,circRadius);

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
      delta_math::calcCubicPoint(te,t,0,theta_end,theta_i,dtheta_i);

      delta_math::rotatePointAroundCircleAxis(p1 - circCenter ,pos_i,theta_i,circAxis);
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
         delta_math::calcCubicPoint(te,t,pos_start[i],pos_end[i],x_i[i],dx_i[i]);
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
