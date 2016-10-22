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
#include "delta_math.hpp"

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


  DeltaPlanner(delta_qtgui::QNode &qnode);
  virtual ~DeltaPlanner();

  int movePTP(std::string posname, float vmax = 35, float stepSize = 0.005,const InterpolationMode &imode = CUBICCART,const MotionMode &mmode = ABSOLUTE);
  int movePTP(std::vector<float> &pos_end, float vmax = 35, float stepSize = 0.005,const InterpolationMode &imode = CUBICCART,const MotionMode &mmode = ABSOLUTE);
  int movePTP_(std::vector<float> &pos_end, float vmax = 35, float stepSize = 0.005,const InterpolationMode &imode = CUBICCART,const MotionMode &mmode = ABSOLUTE);
  int moveCircular(std::vector<std::vector<float> > &circlePoints, float angle = 360, float vmax = 35.0, float stepSize = 0.005);
  int drawYAML(std::string filename,std::vector<float> &start_pos,float vmax = 35,const MotionMode &mmode = RELATIVE);

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
