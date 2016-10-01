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

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
  , qnode(argc,argv)
  ,kinematics(32,80,295,100)
{

	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( !qnode.init() ) {
      showNoMasterMessage();
    }
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


void delta_qtgui::MainWindow::on_Slider_X_valueChanged(int value)
{

   showValues();
   if(ui.contsendCheck->isChecked()) angleGetAndSend();

}
void delta_qtgui::MainWindow::on_Slider_Y_valueChanged(int value)
{

    showValues();
    if(ui.contsendCheck->isChecked()) angleGetAndSend();


}
void delta_qtgui::MainWindow::on_Slider_Z_valueChanged(int value)
{

   showValues();
   if(ui.contsendCheck->isChecked()) angleGetAndSend();
}

void delta_qtgui::MainWindow::showValues(){
  float t1,t2,t3;
  float x,y,z;
  x = (float)ui.Slider_X->value() / 10;
  y = (float)ui.Slider_Y->value() / 10;
  z = (float)ui.Slider_Z->value() / 10;
   std::string s;

  bool status = kinematics.delta_calcInverse(x,y,z,t1,t2,t3);
  std::stringstream ss;
  ss << x;
  ui.label_X->setText(QString::fromStdString(ss.str()));
  ss.str(""); ss << y;
  ui.label_Y->setText(QString::fromStdString(ss.str()));
  ss.str(""); ss << z;
  ui.label_Z->setText(QString::fromStdString(ss.str()));
  ss.precision(2);

  ss.str(""); ss << t1 *180/M_PI << "°";
  s=ss.str();
  ui.lineEd_t1->setText(QString::fromUtf8(s.c_str()));
  ss.str(""); ss << t2*180/M_PI << "°";
  s=ss.str();
  ui.lineEd_t2->setText(QString::fromUtf8(s.c_str()));
  ss.str(""); ss << t3*180/M_PI << "°";
   s=ss.str();
  ui.lineEd_t3->setText(QString::fromUtf8(s.c_str()));
  ss.str(""); ss << status;
 // std::string s =ss.str();
  //ROS_INFO(s.c_str());
}

void delta_qtgui::MainWindow::angleGetAndSend(){
  float t1,t2,t3;
  float x,y,z;
  x = (float)ui.Slider_X->value() / 10;
  y = (float)ui.Slider_Y->value() / 10;
  z = (float)ui.Slider_Z->value() / 10;
  kinematics.delta_calcInverse(x,y,z,t1,t2,t3);
  t1 = t1 *180/M_PI;
  t2 = t2 *180/M_PI;
  t3 = t3 *180/M_PI;
  qnode.sendDeltaAngle(t1,t2,t3,90,90,90);
}

void delta_qtgui::MainWindow::on_sendButton_clicked()
{
   if(ui.cubicCheck->isChecked()){
     float te= ui.durEd->text().toFloat();
     float stepSize= ui.stepsizeEd->text().toFloat();
     goCubic(te,stepSize);
   }
   else{
     angleGetAndSend();
   }

}

void delta_qtgui::MainWindow::on_contsendCheck_clicked(bool checked)
{
    ui.sendButton->setEnabled(!checked);

}

void delta_qtgui::MainWindow::on_getstateButton_clicked()
{
  std::string state = qnode.getDeltaInfo("GETSTATE");
}
void delta_qtgui::MainWindow::goCubic(float te,float stepSize){
  std::string state = qnode.getDeltaInfo("GETSTATE");
  if(state.compare("WAITING")==0){
    float goal_t1,goal_t2,goal_t3;
    float start_t1,start_t2,start_t3;
    float t1, t2, t3;
    float vt1, vt2, vt3;
    float x,y,z;
    x = (float)ui.Slider_X->value() / 10;
    y = (float)ui.Slider_Y->value() / 10;
    z = (float)ui.Slider_Z->value() / 10;
    kinematics.delta_calcInverse(x,y,z,goal_t1,goal_t2,goal_t3);
    goal_t1 = goal_t1 *180/M_PI;
    goal_t2 = goal_t2 *180/M_PI;
    goal_t3 = goal_t3 *180/M_PI;
    qnode.getDeltaAngles("GETANGLES",start_t1,start_t2,start_t3);

    ros::Time endTime = ros::Time::now() + ros::Duration(te);
    ros::Time startTime = ros::Time::now();
    ros::Rate rate(1/stepSize);
    while(ros::ok() && ros::Time::now()<= endTime){
      ros::Duration tr = ros::Time::now() - startTime;
      double t = tr.toSec();
      getCubicAngle(start_t1,goal_t1,te,t,t1,vt1);
      getCubicAngle(start_t2,goal_t2,te,t,t2,vt2);
      getCubicAngle(start_t3,goal_t3,te,t,t3,vt3);
      qnode.sendDeltaAngle(t1,t2,t3,vt1,vt2,vt3);
      ros::spinOnce();
      rate.sleep();
    }
  }
}

void delta_qtgui::MainWindow::getCubicAngle(float qs, float qz, float te, float t, float &q, float &qd){
  q = qs - 3*(qs-qz)/(te * te) * (t*t) + 2*(qs-qz)/(te * te * te) * (t*t*t);
  qd = -6*(qs-qz)/(te * te) * (t) + 6*(qs-qz)/(te * te * te) * (t*t);
  float qdd = -6*(qs-qz)/(te * te) + 12*(qs-qz)/(te * te * te) * t;
  //GEschwindigkeit ist immer mindestens a grad/s beim bremsen
  float a = (float)ui.sliderAngleStep->value();
  if(qdd<0 && 0 < qd && qd < a) qd=a;
  else if(qdd>0 && 0 > qd && qd > -a) qd=-a;

}


void delta_qtgui::MainWindow::on_cubicCheck_clicked(bool checked)
{
        ui.contsendCheck->setEnabled(!checked);
}

void delta_qtgui::MainWindow::on_sliderAngleStep_valueChanged(int value)
{

    ui.sliderAngleLabel->setText(QString::number(ui.sliderAngleStep->value()));
}
