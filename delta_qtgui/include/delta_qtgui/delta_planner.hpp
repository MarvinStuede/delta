#ifndef DELTAPLANNER_H
#define DELTAPLANNER_H
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <delta_kinematics.h>
#include <eigen3/Eigen/Dense>
#include "qnode.hpp"
#include <QObject>
#include "delta_posereader.hpp"


#define XY_LEN			100
#define Z_MAX			-284
#define Z_MIN			-390
#define Z_ROWS			107

class DeltaPlanner : public QObject
{
  Q_OBJECT

public:
  enum InterpolationMode{
    NONE,
    LINEAR,
    CUBICCART
  };
    enum MotionMode{
    ABSOLUTE,
    RELATIVE
  };

  int Workspace[Z_ROWS][XY_LEN][2];
  DeltaPlanner(delta_qtgui::QNode &qnode);
  virtual ~DeltaPlanner();
  void getCubicAngle(float te, float t,const std::vector<float> & q_start, const std::vector<float> & q_end,std::vector<float> & q,std::vector<float> & dq);
  void getCubicCartesian(float te, float t,const std::vector<float> & pos_start, const std::vector<float> & pos_end,std::vector<float> &x, std::vector<float> &dx, std::vector<float> &q,std::vector<float> &qd);
  int readWorkSpace();
  int pointInPolygon(int nvert, float *vertx, float *verty, float testx, float testy);
  int giveBoundedPoint(float &x_prop, float &y_prop, float &z_prop);
  int calcPointOnPolygon(int nvert, float *vertx, float *verty, float testx, float testy,float &lpointx,float &lpointy);
  void rotatePointAroundCircleAxis(const Eigen::Vector3d &p,std::vector<float> &q, double theta,const Eigen::Vector3d &r);
  void calcCubicPoint(float te, float t_i, float p_start, float p_end, float &p_i, float &v_i);
  void calcCircleFromThreePoints(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,const Eigen::Vector3d &p3,Eigen::Vector3d &circCenter,Eigen::Vector3d &circAxis,double &circRadius);
  bool circleInWorkspace(const Eigen::Vector3d &circCenter,const Eigen::Vector3d &circAxis,const double &circRadius);
  int movePTP(std::string posname, float vmax = 35, float stepSize = 0.005,const InterpolationMode &imode = CUBICCART,const MotionMode &mmode = ABSOLUTE);
  int movePTP(std::vector<float> &pos_end, float vmax = 35, float stepSize = 0.005,const InterpolationMode &imode = CUBICCART,const MotionMode &mmode = ABSOLUTE);
  int movePTP_(std::vector<float> &pos_end, float vmax = 35, float stepSize = 0.005,const InterpolationMode &imode = CUBICCART,const MotionMode &mmode = ABSOLUTE);
  int moveCircular(std::vector<std::vector<float> > &circlePoints, float angle = 360, float vmax = 35.0, float stepSize = 0.005);
  int drawYAML(std::string filename,std::vector<float> &start_pos,float vmax = 35);


  bool movFlag;

 public Q_SLOTS:
   void setFlag(const bool en);
 Q_SIGNALS:
   void finished();

private:
  Kinematics kinematics;
  delta_qtgui::QNode* qn;

};

#endif // DELTAPLANNER_H
