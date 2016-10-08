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
void MainWindow::on_lineEd_X_returnPressed()
{
  ui.Slider_X->setValue(ui.lineEd_X->text().toFloat()*10);
  showValues();
  if(ui.contsendCheck->isChecked()) angleGetAndSend();
}
void MainWindow::on_Slider_Y_valueChanged(int value)
{

    showValues();
    if(ui.contsendCheck->isChecked()) angleGetAndSend();


}
void MainWindow::on_lineEd_Y_returnPressed()
{
    ui.Slider_Y->setValue(ui.lineEd_Y->text().toFloat()*10);
  showValues();
  if(ui.contsendCheck->isChecked()) angleGetAndSend();
}
void MainWindow::on_Slider_Z_valueChanged(int value)
{

   showValues();
   if(ui.contsendCheck->isChecked()) angleGetAndSend();
}
void MainWindow::on_lineEd_Z_returnPressed()
{
    ui.Slider_Z->setValue(ui.lineEd_Z->text().toFloat()*10);
  showValues();
  if(ui.contsendCheck->isChecked()) angleGetAndSend();
}

void MainWindow::showValues(){
  std::vector<float> q(3,0);
  std::vector<float> x(3,0);
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

  std::string s;
  bool status = kinematics.delta_calcInverse(x,q);
  std::stringstream ss;
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
 // std::string s =ss.str();
  //ROS_INFO(s.c_str());
}

void MainWindow::angleGetAndSend(){
  std::vector<float> q(3,0);
  std::vector<float> dq(3,90);
  std::vector<float> x(3,0);
      float xi,yi,zi;
  x[0] = (float)ui.Slider_X->value() / 10;
  x[1]  = (float)ui.Slider_Y->value() / 10;
  x[2]  = (float)ui.Slider_Z->value() / 10;
  kinematics.delta_calcInverse(x,q);

  kinematics.rad2deg(q);
  kinematics.delta_calcForward(q,x);
  qnode.sendDeltaCart(x);

 qnode.sendDeltaAngle(q,dq);
}

void MainWindow::on_sendButton_clicked()
{
   if(ui.cubicCheck->isChecked()){
     float te= ui.durEd->text().toFloat();
     float stepSize= ui.stepsizeEd->text().toFloat();
     if(ui.cartesianCheck->isChecked()){
       goCubicCart(te,stepSize);
     }
     else{
       goCubic(te,stepSize);
     }
   }
   else{
     angleGetAndSend();
   }

}


void MainWindow::on_resetTopButton_clicked()
{
  ui.Slider_X->setValue(0);
  ui.Slider_Y->setValue(0);
  ui.Slider_Z->setValue(-284 * 10);
  showValues();
}

void MainWindow::on_contsendCheck_clicked(bool checked)
{
    ui.sendButton->setEnabled(!checked);
}

void MainWindow::goCubic(float te,float stepSize){
  std::string state = qnode.getDeltaInfo("GETSTATE");
  if(state.compare("WAITING")==0){
    std::vector<float> q_end(3,0);
    std::vector<float> q_start(3,0);
    std::vector<float> q(3,0);
    std::vector<float> dq(3,0);
    std::vector<float> pos_end(3,0);
    std::vector<float> xi(3,0);

    pos_end[0] = (float)ui.Slider_X->value() / 10;
    pos_end[1] = (float)ui.Slider_Y->value() / 10;
    pos_end[2] = (float)ui.Slider_Z->value() / 10;

    kinematics.delta_calcInverse(pos_end,q_end);
    kinematics.rad2deg(q_end);
    qnode.getDeltaAngles("GETANGLES", q_start);

    ros::Time endTime = ros::Time::now() + ros::Duration(te);
    ros::Time startTime = ros::Time::now();
    ros::Rate rate(1/stepSize);

    float a = (float)ui.sliderAngleStep->value();
    std::vector<float> dq_end(3,a);

    while(ros::ok() && ros::Time::now()<= endTime){
      ros::Duration tr = ros::Time::now() - startTime;
      double t = tr.toSec();
      deltaplanner.getCubicAngle(te,t,q_start,q_end,dq_end,q,dq);

      kinematics.delta_calcForward(q,xi);
      qnode.sendDeltaCart(xi);
      qnode.sendDeltaAngle(q,dq);

      ros::spinOnce();
      rate.sleep();
    }
  }
}

void MainWindow::goCubicCart(float te,float stepSize){
  std::string state = qnode.getDeltaInfo("GETSTATE");
  if(state.compare("WAITING")==0){


    float t1, t2, t3;
    float vt1, vt2, vt3;

    std::vector<float> pos_start(3,0);
    std::vector<float> q_start(3,0);
    std::vector<float> pos_end(3,0);
    std::vector<float> xi(3,0);
    std::vector<float> dxi(3,0);
    std::vector<float> q(3,0);
    std::vector<float> dq(3,0);
    pos_end[0] = (float)ui.Slider_X->value() / 10;
    pos_end[1] = (float)ui.Slider_Y->value() / 10;
    pos_end[2] = (float)ui.Slider_Z->value() / 10;


    qnode.getDeltaAngles("GETANGLES",q_start);
    kinematics.delta_calcForward(q_start,pos_start);
    ros::Time endTime = ros::Time::now() + ros::Duration(te);
    ros::Time startTime = ros::Time::now();
    ros::Rate rate(1/stepSize);

    while(ros::ok() && ros::Time::now()<= endTime){
      ros::Duration tr = ros::Time::now() - startTime;
      double t = tr.toSec();

      deltaplanner.getCubicCartesian(te,t,pos_start,pos_end,xi,dxi,q,dq);

      qnode.sendDeltaCart(xi);
      qnode.sendDeltaVel(dq);
      kinematics.rad2deg(q);
      qnode.sendDeltaAngle(q,dq);
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


