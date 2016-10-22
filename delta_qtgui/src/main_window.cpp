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
  ,deltaplanner(qnode)
  ,wswatchdog()
{

	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    //Connection für anzeige aktueller Gelenkwinkel
    QObject::connect(&qnode, SIGNAL(JointStateUpdated(float, float, float)), this, SLOT(updateJointState(float, float, float)));

    /*********************
    ** Auto Start
    **********************/
    if ( !qnode.init() ) {
      showNoMasterMessage();
    }
    if(!wswatchdog.readWorkSpace()){
      qnode.log(QNode::Error,"Workspace Data not loaded");
    }
    else{
     qnode.log(QNode::Info,"Workspace Data loaded");
    }

    showValues();
    //Tastaturshortcuts
    QShortcut *scM = new QShortcut(QKeySequence("M"), this);
    connect(scM, SIGNAL(activated()), this, SLOT(on_sendButton_clicked()));
    QShortcut *scS = new QShortcut(QKeySequence("S"), this);
    connect(scS, SIGNAL(activated()), this, SLOT(on_stopButton_clicked()));
    QShortcut *scE = new QShortcut(QKeySequence("E"), this);
    connect(scE, SIGNAL(activated()), this, SLOT(on_enableButton_clicked()));
    QShortcut *scH = new QShortcut(QKeySequence("H"), this);
    connect(scH, SIGNAL(activated()), this, SLOT(on_homeButton_clicked()));

    //Thread für Bewegung initialisieren
    movThread = new QThread;
    deltaMov = new DeltaPlanner(qnode);
    deltaMov->moveToThread(movThread);
    connect(deltaMov, SIGNAL(finished()), movThread, SLOT(quit()));
    connect(deltaMov, SIGNAL(finished()), deltaMov, SLOT(deleteLater()));
    connect(this,SIGNAL(setMovFlag(const bool)),deltaMov,SLOT(setFlag(const bool)));

    movThread->start();

    //Geteachte Punkte einlesen
    poses=delta_posereader::read();
    setTeachedPoint(poses.begin()->first);
    setPoseListEntries();


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
** Implementation [Slots][manually connected]
*****************************************************************************/

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
** MainWindow Refresh Funktion
*****************************************************************************/
void MainWindow::showValues(){

  vector<float> q(3,0);
  vector<float> x(3,0);
  x[0] = (float)ui.Slider_X->value() / 10;
  x[1] = (float)ui.Slider_Y->value() / 10;
  x[2] = (float)ui.Slider_Z->value() / 10;


  int boundReturn = wswatchdog.giveBoundedPoint(x[0],x[1],x[2]);
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
  delta_math::calcCircleFromThreePoints(p1,p2,p3,circCenter,circAxis,circRadius);

  if(wswatchdog.circleInWorkspace(circCenter,circAxis,circRadius)){
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
void MainWindow::goPTP(const DeltaPlanner::InterpolationMode &mode){
  //Funktion zum Aufruf einer PTP Bewegung
    vector<float> pos_end(3,0);
    pos_end[0] = (float)ui.Slider_X->value() / 10;
    pos_end[1]  = (float)ui.Slider_Y->value() / 10;
    pos_end[2]  = (float)ui.Slider_Z->value() / 10;

    float vmax= ui.speedBox->text().toFloat();
    float stepSize= ui.stepsizeEd->text().toFloat();

    if(mode == DeltaPlanner::NONE)
      deltaMov->movePTP(pos_end,vmax,stepSize,mode);
    else{
      //Nur fahren wenn Roboter bereit
      string state = qnode.getDeltaInfo("GETSTATE");
      if(state.compare("WAITING")==0)
        deltaMov->movePTP(pos_end,vmax,stepSize,mode);
    }
}
void MainWindow::goCircular(){
  //Funktion zum Aufruf einer Kreisbewegung
    vector<vector<float> > circlePoints(3, vector<float>(3));
    float angle = ui.led_angle->text().toFloat();
    circlePoints[0][0] = ui.led_CP1_X->text().toFloat();
    circlePoints[0][1] = ui.led_CP1_Y->text().toFloat();
    circlePoints[0][2] = ui.led_CP1_Z->text().toFloat();
    circlePoints[1][0] = ui.led_CP2_X->text().toFloat();
    circlePoints[1][1] = ui.led_CP2_Y->text().toFloat();
    circlePoints[1][2] = ui.led_CP2_Z->text().toFloat();
    circlePoints[2][0] = ui.led_CP3_X->text().toFloat();
    circlePoints[2][1] = ui.led_CP3_Y->text().toFloat();
    circlePoints[2][2] = ui.led_CP3_Z->text().toFloat();

    float vmax= ui.speedBox->text().toFloat();
    float stepSize= ui.stepsizeEd->text().toFloat();
    string state = qnode.getDeltaInfo("GETSTATE");
    if(state.compare("WAITING")==0)
      deltaMov->moveCircular(circlePoints,angle,vmax,stepSize);

}

void MainWindow::stopMotion(){
  qnode.sendDeltaCmd("STOPMOVE");
}
void MainWindow::setTeachedPoint(std::string pointname){
   //Slider auf Wert von Comboboxelement setzen
     ui.Slider_X->setValue(poses[pointname].x*10);
     ui.Slider_Y->setValue(poses[pointname].y*10);
     ui.Slider_Z->setValue(poses[pointname].z*10);
     showValues();
}
void MainWindow::setPoseListEntries(){
//Listboxeinträge auf Map (YAML) Inhalt setzen
     ui.listPoses->clear();
    for(delta_posereader::PoseMap::iterator it=poses.begin();it!=poses.end();++it){
      ui.listPoses->addItem(QString::fromStdString(it->first));
    }

}

/*****************************************************************************
** CALLBACKS
*****************************************************************************/
//########### BUTTONS #############################
void MainWindow::on_disableButton_clicked(){
  qnode.sendDeltaCmd("CTRLSTOP");
}
void MainWindow::on_enableButton_clicked(){
  qnode.sendDeltaCmd("CTRLSTART");
}
void MainWindow::on_sendButton_clicked()
{

   if(ui.radioCartCubic->isChecked()){
     if(!(deltaMov->movFlag)) goPTP(DeltaPlanner::CUBICCART);
   }
   else if (ui.radioLinear->isChecked()){
     goPTP(DeltaPlanner::LINEAR);
   }
   else if(ui.radioCircular->isChecked()){
     goCircular();
   }

}
void MainWindow::on_resetButton_clicked(){
    qnode.sendDeltaCmd("RESET");
    ui.Slider_X->setValue(0);
    ui.Slider_Y->setValue(0);
    ui.Slider_Z->setValue(-253 * 10);
    showValues();
}
void MainWindow::on_homeButton_clicked()
{
      qnode.sendDeltaCmd("TURNOFF");
}
void MainWindow::on_houseButton_clicked()
{
  float vmax= ui.speedBox->text().toFloat();
  float stepSize= ui.stepsizeEd->text().toFloat();
  //goHouse(vmax,stepSize);
}
void MainWindow::on_stopButton_clicked()
{
   setMovFlag(false);
   stopMotion();
}
void MainWindow::on_resetTopButton_clicked()
{
  ui.Slider_X->setValue(0);
  ui.Slider_Y->setValue(0);
  ui.Slider_Z->setValue(-253 * 10);
  showValues();
}
void MainWindow::on_gripopenButton_clicked()
{
  qnode.sendDeltaCmd("GRIPPEROPEN");
}
void MainWindow::on_gripcloseButton_clicked()
{
  qnode.sendDeltaCmd("GRIPPERCLOSE");
}
void MainWindow::on_button_teachPoint_clicked()
{
  //Aktuellen Punkt map hinzufügen und speichern

  std::string pointname = ui.led_pointName->text().toStdString();
  geometry_msgs::Point point;
  point.x = (float)ui.Slider_X->value() / 10;
  point.y = (float)ui.Slider_Y->value() / 10;
  point.z = (float)ui.Slider_Z->value() / 10;
  poses.insert(poses.end(),std::pair<std::string,geometry_msgs::Point>(pointname,point));
  delta_posereader::write(poses);
  setPoseListEntries();
  ui.button_teachPoint->setEnabled(false);
  //deltaMov->movePTP(pointname);

}
void MainWindow::on_button_drawCross_clicked()
{
    std::vector<float> pos(3,0);
    pos[0] = (float)ui.Slider_X->value() / 10;
    pos[1] = (float)ui.Slider_Y->value() / 10;
    pos[2] = (float)ui.Slider_Z->value() / 10;
    float vmax = ui.speedBox->text().toFloat();
    deltaMov->drawYAML("cross.yaml",pos,vmax);
}

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Delta QtGUI</h2><p>Copyright Marvin Stüde/p><p>-</p>"));
}
void MainWindow::on_Slider_X_valueChanged(int value)
{

   showValues();
   if(ui.radioContinuous->isChecked()) goPTP(DeltaPlanner::NONE);

}
void MainWindow::on_lineEd_X_returnPressed()
{
  ui.Slider_X->setValue(ui.lineEd_X->text().toFloat()*10);
  showValues();
  if(ui.radioContinuous->isChecked()) goPTP(DeltaPlanner::NONE);
}
void MainWindow::on_Slider_Y_valueChanged(int value)
{
    showValues();
    if(ui.radioContinuous->isChecked()) goPTP(DeltaPlanner::NONE);

}
void MainWindow::on_lineEd_Y_returnPressed()
{
    ui.Slider_Y->setValue(ui.lineEd_Y->text().toFloat()*10);
  showValues();
  if(ui.radioContinuous->isChecked()) goPTP(DeltaPlanner::NONE);
}
void MainWindow::on_Slider_Z_valueChanged(int value)
{

   showValues();
   if(ui.radioContinuous->isChecked()) goPTP(DeltaPlanner::NONE);
}
void MainWindow::on_lineEd_Z_returnPressed()
{
    ui.Slider_Z->setValue(ui.lineEd_Z->text().toFloat()*10);
  showValues();
  if(ui.radioContinuous->isChecked()) goPTP(DeltaPlanner::NONE);
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
void MainWindow::on_led_radius_returnPressed()
{
    showValues();
}
void MainWindow::on_led_pointName_textEdited(const QString &arg1)
{
  //Button zum teachen nur aktivieren wenn Name noch nicht vorhanden
  ui.button_teachPoint->setEnabled(!(poses.find(arg1.toStdString()) != poses.end()));

}
void MainWindow::on_listPoses_itemClicked(QListWidgetItem *item)
{
       setTeachedPoint(item->text().toStdString());
}
//########### RADIO BUTTONS #############################
void MainWindow::on_radioCircular_clicked()
{
    showValues();
}
void MainWindow::on_radioCartCubic_clicked()
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
  movThread->quit();
	QMainWindow::closeEvent(event);

}

}  // namespace delta_qtgui

