/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/delta_qtgui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delta_qtgui {

using namespace Qt;
using namespace std;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
  , qnode(argc,argv)
  ,kinematics(32,80,295,100)
  ,deltaplanner()
{

	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));



    QObject::connect(&qnode, SIGNAL(JointStateUpdated(float, float, float)), this, SLOT(updateJointState(float, float, float)));

    /*********************
    ** Auto Start
    **********************/
    if ( !qnode.init() ) {
      showNoMasterMessage();
    }
    while(!(deltaplanner.readWorkSpace()==0));
    showValues();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}
void MainWindow::updateJointState(float q0, float q1, float q2){
  stringstream ss;
  ss.precision(4);
  ss.str(""); ss << q0 << "°";
  string s=ss.str();
  ui.lineEd_t1_ist->setText(QString::fromUtf8(s.c_str()));
  ss.str(""); ss << q1 << "°";
  s=ss.str();
  ui.lineEd_t2_ist->setText(QString::fromUtf8(s.c_str()));
  ss.str(""); ss << q2 << "°";
   s=ss.str();
  ui.lineEd_t3_ist->setText(QString::fromUtf8(s.c_str()));
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}
void MainWindow::on_disableButton_clicked()
{
    qnode.sendDeltaCmd("CTRLSTOP");

}
void MainWindow::on_enableButton_clicked()
{
    qnode.sendDeltaCmd("CTRLSTART");
}
void MainWindow::on_resetButton_clicked()
{
    qnode.sendDeltaCmd("RESET");
    ui.Slider_X->setValue(0);
    ui.Slider_Y->setValue(0);
    ui.Slider_Z->setValue(-284 * 10);
    showValues();
}
void MainWindow::on_Slider_X_valueChanged(int value)
{

   showValues();
   if(ui.radioContinuous->isChecked()) goUncoordinatedLinear();

}
void MainWindow::on_lineEd_X_returnPressed()
{
  ui.Slider_X->setValue(ui.lineEd_X->text().toFloat()*10);
  showValues();
  if(ui.radioContinuous->isChecked()) goUncoordinatedLinear();
}
void MainWindow::on_Slider_Y_valueChanged(int value)
{

    showValues();
    if(ui.radioContinuous->isChecked()) goUncoordinatedLinear();


}
void MainWindow::on_lineEd_Y_returnPressed()
{
    ui.Slider_Y->setValue(ui.lineEd_Y->text().toFloat()*10);
  showValues();
  if(ui.radioContinuous->isChecked()) goUncoordinatedLinear();
}
void MainWindow::on_Slider_Z_valueChanged(int value)
{

   showValues();
   if(ui.radioContinuous->isChecked()) goUncoordinatedLinear();
}
void MainWindow::on_lineEd_Z_returnPressed()
{
    ui.Slider_Z->setValue(ui.lineEd_Z->text().toFloat()*10);
  showValues();
  if(ui.radioContinuous->isChecked()) goUncoordinatedLinear();
}

void MainWindow::on_houseButton_clicked()
{
  float vmax= ui.speedBox->text().toFloat();
  float stepSize= ui.stepsizeEd->text().toFloat();
  goHouse(vmax,stepSize);
}
void MainWindow::on_pushButton_clicked()
{
      qnode.sendDeltaCmd("TURNOFF");
}
void MainWindow::on_speedSlider_valueChanged(int value)
{
    stringstream ss;
    ss << value;
    ui.speedBox->setText(QString::fromStdString(ss.str()));
}
void MainWindow::on_speedBox_returnPressed()
{
    ui.speedSlider->setValue( ui.speedBox->text().toFloat());
}
void MainWindow::on_sendButton_clicked()
{
   float vmax= ui.speedBox->text().toFloat();
   float stepSize= ui.stepsizeEd->text().toFloat();
   if(ui.radioCartCubic->isChecked() || ui.radioAngCubic->isChecked()){
     goCubic(vmax,stepSize);
   }
   else if (ui.radioLinear->isChecked()){
     goCoordinatedLinear(vmax);
   }
   else if(ui.radioCircular->isChecked()){
     goCircular(vmax,stepSize);
   }

}
void MainWindow::on_resetTopButton_clicked()
{
  ui.Slider_X->setValue(0);
  ui.Slider_Y->setValue(0);
  ui.Slider_Z->setValue(-284 * 10);
  showValues();
}

void MainWindow::showValues(){

  vector<float> q(3,0);
  vector<float> x(3,0);
  x[0] = (float)ui.Slider_X->value() / 10;
  x[1] = (float)ui.Slider_Y->value() / 10;
  x[2] = (float)ui.Slider_Z->value() / 10;


  int boundReturn = deltaplanner.giveBoundedPoint(x[0],x[1],x[2]);
  if(boundReturn==1){
     ui.label_workspace->setText("In Workspace");
     ui.label_workspace->setStyleSheet("QLabel {color : green}");
  }
  else if(boundReturn==2){
    ui.label_workspace->setText("At Workspace boundary");
    ui.label_workspace->setStyleSheet("QLabel {color : #fca016}");// orange
    ui.Slider_X->setValue((int)x[0]*10);
    ui.Slider_Y->setValue((int)x[1]*10);
    ui.Slider_Z->setValue((int)x[2]*10);

  }
  else{
    ui.label_workspace->setText("Error: Out of Workspace");
    ui.label_workspace->setStyleSheet("color: red}");
  }




  string s;
  bool status = kinematics.delta_calcInverse(x,q);
  stringstream ss;
  ss << x[0];
  ui.lineEd_X->setText(QString::fromStdString(ss.str()));
  ss.str(""); ss << x[1];
  ui.lineEd_Y->setText(QString::fromStdString(ss.str()));
  ss.str(""); ss << x[2];
  ui.lineEd_Z->setText(QString::fromStdString(ss.str()));
  ss.precision(4);

  ss.str(""); ss << q[0] *180/M_PI << "°";
  s=ss.str();
  ui.lineEd_t1->setText(QString::fromUtf8(s.c_str()));
  ss.str(""); ss << q[1]*180/M_PI << "°";
  s=ss.str();
  ui.lineEd_t2->setText(QString::fromUtf8(s.c_str()));
  ss.str(""); ss << q[2]*180/M_PI << "°";
   s=ss.str();
  ui.lineEd_t3->setText(QString::fromUtf8(s.c_str()));
  ss.str(""); ss << status;


  //##CIRCLE###################
  float r =  ui.led_radius->text().toFloat();

  ui.led_CP1_X->setText(ui.lineEd_X->text());
  ui.led_CP1_Y->setText(ui.lineEd_Y->text());
  ui.led_CP1_Z->setText(ui.lineEd_Z->text());

  ss.str("");
  ss<<ui.lineEd_X->text().toFloat();
  ui.led_CP2_X->setText(QString::fromStdString(ss.str()));
  ss.str("");
  ss<<ui.lineEd_Y->text().toFloat() - 2 * r;
  ui.led_CP2_Y->setText(QString::fromStdString(ss.str()));
  ss.str("");
  ss<<ui.lineEd_Z->text().toFloat();
  ui.led_CP2_Z->setText(QString::fromStdString(ss.str()));

  ss.str("");
  ss<<ui.lineEd_X->text().toFloat() + r;
  ui.led_CP3_X->setText(QString::fromStdString(ss.str()));
  ss.str("");
  ss<<ui.lineEd_Y->text().toFloat() - r;
  ui.led_CP3_Y->setText(QString::fromStdString(ss.str()));
  ss.str("");
  ss<<ui.lineEd_Z->text().toFloat();
  ui.led_CP3_Z->setText(QString::fromStdString(ss.str()));

  using namespace Eigen;
  Vector3d circCenter;
  Vector3d circAxis;
  double circRadius;
  Vector3d p1(ui.led_CP1_X->text().toFloat(),ui.led_CP1_Y->text().toFloat(),ui.led_CP1_Z->text().toFloat());
  Vector3d p2(ui.led_CP2_X->text().toFloat(),ui.led_CP2_Y->text().toFloat(),ui.led_CP2_Z->text().toFloat());
  Vector3d p3(ui.led_CP3_X->text().toFloat(),ui.led_CP3_Y->text().toFloat(),ui.led_CP3_Z->text().toFloat());
  deltaplanner.calcCircleFromThreePoints(p1,p2,p3,circCenter,circAxis,circRadius);
  if(deltaplanner.circleInWorkspace(circCenter,circAxis,circRadius)){
    ui.label_workspace_circle->setText("Circle in Workspace");
    ui.label_workspace_circle->setStyleSheet("QLabel {color : green}");
    ui.sendButton->setEnabled(true);
  }
  else{
       ui.label_workspace_circle->setText("Circle not in Workspace");
       ui.label_workspace_circle->setStyleSheet("QLabel {color : red}");
       ui.sendButton->setEnabled(false);
  }

  if(!ui.radioCircular->isChecked()){
      ui.sendButton->setEnabled(true);
  }

}

void MainWindow::goCoordinatedLinear(float v){
  //Synchronisierte Linearbewegung mit konstanter Geschwindigkeit
  vector<float> q_goal(3,0);
  vector<float> dq(3,90);
  vector<float> x(3,0);
  vector<float> q_start(3,0);
  float tmax = 0;
  vector<float> dist(3,0);

  x[0] = (float)ui.Slider_X->value() / 10;
  x[1]  = (float)ui.Slider_Y->value() / 10;
  x[2]  = (float)ui.Slider_Z->value() / 10;

  qnode.getDeltaAngles("GETANGLES", q_start);
  kinematics.delta_calcInverse(x,q_goal);
    kinematics.rad2deg(q_goal);
  for(int i = 0;i<3;i++){
      dist[i]=fabs(q_goal[i] - q_start[i]);

      if (dist[i] / v > tmax) tmax = dist[i]/v;
  }

  dq[0]=dist[0]/tmax;
  dq[1]=dist[1]/tmax;
  dq[2]=dist[2]/tmax;
  qnode.sendDeltaCart(x);

 qnode.sendDeltaAngle(q_goal,dq);
}
void MainWindow::goUncoordinatedLinear(){
  //Unsynchronisierte Linearbewegung (benutzt für z.B. cont. sending)

  vector<float> q_goal(3,0);
  vector<float> dq(3,90);
  vector<float> x(3,0);

  x[0] = (float)ui.Slider_X->value() / 10;
  x[1]  = (float)ui.Slider_Y->value() / 10;
  x[2]  = (float)ui.Slider_Z->value() / 10;

  kinematics.delta_calcInverse(x,q_goal);
  kinematics.rad2deg(q_goal);
  qnode.sendDeltaCart(x);
  qnode.sendDeltaAngle(q_goal,dq);

}
void MainWindow::goCubic(float vmax,float stepSize){
  string state = qnode.getDeltaInfo("GETSTATE");

  if(state.compare("WAITING")==0){
    vector<float> q_end(3,0);
    vector<float> q_start(3,0);
    vector<float> q(3,0);
    vector<float> dq(3,0);
    vector<float> pos_start(3,0);
    vector<float> pos_end(3,0);
    vector<float> xi(3,0);
    vector<float> dxi(3,0);

    float te = 0;

    pos_end[0] = (float)ui.Slider_X->value() / 10;
    pos_end[1] = (float)ui.Slider_Y->value() / 10;
    pos_end[2] = (float)ui.Slider_Z->value() / 10;

    qnode.getDeltaAngles("GETANGLES", q_start);

    bool moveCart = ui.radioCartCubic->isChecked();
    kinematics.delta_calcInverse(pos_end,q_end);
    kinematics.delta_calcForward(q_start,pos_start);
    kinematics.rad2deg(q_end);

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
    vector<float> dq_end(3,0);

    while(ros::ok() && ros::Time::now()<= endTime){
      ros::Duration tr = ros::Time::now() - startTime;
      double t = tr.toSec();

      if(moveCart){
        deltaplanner.getCubicCartesian(te,t,pos_start,pos_end,xi,dxi,q,dq);
        kinematics.rad2deg(q);
      }
      else{

        deltaplanner.getCubicAngle(te,t,q_start,q_end,q,dq);
        kinematics.delta_calcForward(q,xi);
      }

      dq[0]=40;
      dq[1]=40;
      dq[2]=40;
      qnode.sendDeltaCart(xi);
      qnode.sendDeltaAngle(q,dq);
      ros::spinOnce();

      qApp->processEvents();
      if(ui.stopButton->isDown()) break;

      rate.sleep();
    }
  }
}

bool MainWindow::goCircular(float vmax, float stepSize){

  vector<float> q_start(3,0);
  vector<float> pos_start(3,0);
  vector<float> dq(3,0);
  vector<float> q(3,0);
  string state = qnode.getDeltaInfo("GETSTATE");

  if(state.compare("WAITING")==0){
    qnode.getDeltaAngles("GETANGLES", q_start);
    kinematics.delta_calcForward(q_start,pos_start);

    using namespace Eigen;

     //Drei Punkte auf Kreis
    Vector3d p1(ui.led_CP1_X->text().toFloat(),ui.led_CP1_Y->text().toFloat(),ui.led_CP1_Z->text().toFloat());
    Vector3d p2(ui.led_CP2_X->text().toFloat(),ui.led_CP2_Y->text().toFloat(),ui.led_CP2_Z->text().toFloat());
    Vector3d p3(ui.led_CP3_X->text().toFloat(),ui.led_CP3_Y->text().toFloat(),ui.led_CP3_Z->text().toFloat());
    Vector3d circCenter;
    Vector3d circAxis;
    double circRadius;

    deltaplanner.calcCircleFromThreePoints(p1,p2,p3,circCenter,circAxis,circRadius);
    //Punkt auf Kreis in aktueller Iteration
    vector<float> pos_i(3,0);

    //Winkel der gefahren werden soll
    float theta_end = 360 * M_PI/180;
    float theta_i = 0;
    float dtheta_i = 0;

    //Gesamtdauer für Kreisfahrt
    float te = fabs(1.5*theta_end*circRadius/vmax);

    ros::Time endTime = ros::Time::now() + ros::Duration(te);
    ros::Time startTime = ros::Time::now();
    ros::Rate rate(1/stepSize);
    vector<float> xi(3,0);

    while(ros::ok() && ros::Time::now()<= endTime){
      ros::Duration tr = ros::Time::now() - startTime;
      double t = tr.toSec();
      //Gebe Punkt auf Kreis an bestimmtem Winkel zurück (Im Kreiskoord.-system)
      deltaplanner.calcCubicPoint(te,t,0,theta_end,theta_i,dtheta_i);

      deltaplanner.rotatePointAroundCircleAxis(p1 - circCenter ,pos_i,theta_i,circAxis);
      //In Taskspace transformieren
      for(int i=0;i<3;i++) xi[i]= pos_i[i] + circCenter[i];

      qnode.sendDeltaCart(xi);
      dq[0]=40;
      dq[1]=40;
      dq[2]=40;

      kinematics.delta_calcInverse(xi,q);
      kinematics.rad2deg(q);
      qnode.sendDeltaAngle(q,dq);
      ros::spinOnce();
      qApp->processEvents();
      if(ui.stopButton->isDown()) break;

      rate.sleep();
    }

  }
}

void MainWindow::goHouse(float vmax,float stepSize){
  string state = qnode.getDeltaInfo("GETSTATE");

  if(state.compare("WAITING")==0){
    vector<float> q_end(3,0);
    vector<float> q_start(3,0);
    vector<float> q(3,0);
    vector<float> dq(3,0);
    vector<float> pos_start(3,0);
    vector<float> pos_end(3,0);
    vector<float> xi(3,0);
    vector<float> dxi(3,0);
    vector< vector<float> > arr(10, vector<float>(3));
    stringstream ss;

    arr[0][0]=-30;
    arr[0][1]=-30;
    arr[0][2]=-310.3;

    arr[1][0]=30;
    arr[1][1]=-30;
    arr[1][2]=-310.3;

    arr[2][0]=-30;
    arr[2][1]=30;
    arr[2][2]=-310.3;

    arr[3][0]=30;
    arr[3][1]=30;
    arr[3][2]=-310.3;

    arr[4][0]=0;
    arr[4][1]=60;
    arr[4][2]=-310.3;

    arr[5][0]=-30;
    arr[5][1]=30;
    arr[5][2]=-310.3;

    arr[6][0]=-30;
    arr[6][1]=-30;
    arr[6][2]=-310.3;

    arr[7][0]=30;
    arr[7][1]=30;
    arr[7][2]=-310.3;

    arr[8][0]=30;
    arr[8][1]=-30;
    arr[8][2]=-310.3;

    arr[9][0]=0;
    arr[9][1]=0;
    arr[9][2]=-284;


    float te = 0;
    for(int p = 0; p<10;p++){
          pos_end[0] = arr[p][0];
          pos_end[1] = arr[p][1];
          pos_end[2] = arr[p][2];
          ss.str("");
          ss << "Moving to x: "<<pos_end[0]<<", y: "<<pos_end[1]<<", z: "<<pos_end[2];
          qnode.log(QNode::Info,ss.str());
          qnode.getDeltaAngles("GETANGLES", q_start);

          bool moveCart = ui.radioCartCubic->isChecked();
          kinematics.delta_calcInverse(pos_end,q_end);
          kinematics.delta_calcForward(q_start,pos_start);
          kinematics.rad2deg(q_end);

          float tj = 0;
          for(int j=0;j<3;j++){
              tj=fabs(1.5*(q_end[j] - q_start[j]) / vmax);
              if (tj > te) te = tj;
           }

          ros::Time endTime = ros::Time::now() + ros::Duration(te);
          ros::Time startTime = ros::Time::now();
          ros::Rate rate(1/stepSize);

          vector<float> dq_end(3,0);

          while(ros::ok() && ros::Time::now()<= endTime){
            ros::Duration tr = ros::Time::now() - startTime;
            double t = tr.toSec();

            if(moveCart){
              deltaplanner.getCubicCartesian(te,t,pos_start,pos_end,xi,dxi,q,dq);
              kinematics.rad2deg(q);
            }
            else{

              deltaplanner.getCubicAngle(te,t,q_start,q_end,q,dq);
              kinematics.delta_calcForward(q,xi);
            }

            qnode.sendDeltaCart(xi);
            dq[0]=40;
            dq[1]=40;
            dq[2]=40;
            qnode.sendDeltaAngle(q,dq);
            ros::spinOnce();
            qApp->processEvents();
            if(ui.stopButton->isDown()){
             //Beide schleifen verlassen
             p=10;
             break;
             }
            rate.sleep();
          }
        }


     ros::Duration(0.5).sleep();
  }
}

void MainWindow::stopMotion(){
  qnode.sendDeltaCmd("STOPMOVE");
}
void MainWindow::on_stopButton_clicked()
{
   stopMotion();
}
void MainWindow::on_gripopenButton_clicked()
{
  qnode.sendDeltaCmd("GRIPPEROPEN");
}

void MainWindow::on_gripcloseButton_clicked()
{
  qnode.sendDeltaCmd("GRIPPERCLOSE");
}
void MainWindow::on_led_radius_returnPressed()
{
    showValues();
}
void MainWindow::on_radioCircular_clicked()
{
    showValues();
}

void MainWindow::on_radioCartCubic_clicked()
{
    showValues();
}

void MainWindow::on_radioAngCubic_clicked()
{
    showValues();
}

void MainWindow::on_radioLinear_clicked()
{
    showValues();
}

void MainWindow::on_radioContinuous_clicked()
{
    showValues();
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "delta_qtgui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    bool checked = settings.value("use_environment_variables", false).toBool();

}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "delta_qtgui");
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace delta_qtgui

