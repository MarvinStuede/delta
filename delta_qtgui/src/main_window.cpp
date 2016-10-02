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
  ,deltaplanner()
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

void MainWindow::on_Slider_X_valueChanged(int value)
{

   showValues();
   if(ui.contsendCheck->isChecked()) angleGetAndSend();

}
void MainWindow::on_Slider_Y_valueChanged(int value)
{

    showValues();
    if(ui.contsendCheck->isChecked()) angleGetAndSend();


}
void MainWindow::on_Slider_Z_valueChanged(int value)
{

   showValues();
   if(ui.contsendCheck->isChecked()) angleGetAndSend();
}

void MainWindow::showValues(){
  float t1,t2,t3;
  float x,y,z;
  x = (float)ui.Slider_X->value() / 10;
  y = (float)ui.Slider_Y->value() / 10;
  z = (float)ui.Slider_Z->value() / 10;


  int boundReturn = deltaplanner.giveBoundedPoint(x,y,z);
  if(boundReturn==1){
     ui.label_workspace->setText("In Workspace");
     ui.label_workspace->setStyleSheet("QLabel {color : green}");
  }
  else if(boundReturn==2){
    ui.label_workspace->setText("At Workspace boundary");
    ui.label_workspace->setStyleSheet("QLabel {color : #fca016}");// orange
    ui.Slider_X->setValue((int)x*10);
    ui.Slider_Y->setValue((int)y*10);
    ui.Slider_Z->setValue((int)z*10);

  }
  else{
    ui.label_workspace->setText("Error: Out of Workspace");
    ui.label_workspace->setStyleSheet("color: red}");
  }

  std::string s;
  bool status = kinematics.delta_calcInverse(x,y,z,t1,t2,t3);
  std::stringstream ss;
  ss << x;
  ui.label_X->setText(QString::fromStdString(ss.str()));
  ss.str(""); ss << y;
  ui.label_Y->setText(QString::fromStdString(ss.str()));
  ss.str(""); ss << z;
  ui.label_Z->setText(QString::fromStdString(ss.str()));
  ss.precision(4);

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

void MainWindow::angleGetAndSend(){
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

void MainWindow::on_sendButton_clicked()
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

void MainWindow::on_contsendCheck_clicked(bool checked)
{
    ui.sendButton->setEnabled(!checked);
}

void MainWindow::goCubic(float te,float stepSize){
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

    float a = (float)ui.sliderAngleStep->value();
    while(ros::ok() && ros::Time::now()<= endTime){
      ros::Duration tr = ros::Time::now() - startTime;
      double t = tr.toSec();
      deltaplanner.getCubicAngle(start_t1,goal_t1,te,t,a,t1,vt1);
      deltaplanner.getCubicAngle(start_t2,goal_t2,te,t,a,t2,vt2);
      deltaplanner.getCubicAngle(start_t3,goal_t3,te,t,a,t3,vt3);
      qnode.sendDeltaAngle(t1,t2,t3,vt1,vt2,vt3);
      ros::spinOnce();
      rate.sleep();
    }
  }
}

void MainWindow::on_cubicCheck_clicked(bool checked)
{
        ui.contsendCheck->setEnabled(!checked);
}

void MainWindow::on_sliderAngleStep_valueChanged(int value)
{

    ui.sliderAngleLabel->setText(QString::number(ui.sliderAngleStep->value()));
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
