/**
 * @file /include/delta_qtgui/main_window.hpp
 *
 * @brief Qt based gui for delta_qtgui.
 *
 * @date November 2010
 **/
#ifndef delta_qtgui_MAIN_WINDOW_H
#define delta_qtgui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "Kinematics.h"
#include "deltaplanner.h"
#include <vector>
#include <eigen3/Eigen/Dense>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace delta_qtgui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
  void goCoordinatedLinear(float v);
  void goUncoordinatedLinear();
  void getCubicAngle(float qs, float qz, float te, float t, float ve,float &q, float &qd);
  void goCubic(float vmax,float stepSize);
  void goHouse(float vmax,float stepSize);
  bool goCircular(float vmax,float stepSize);


public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();

  void on_resetButton_clicked();

  void on_disableButton_clicked();

  void on_enableButton_clicked();
  void on_Slider_X_valueChanged(int value);
  void on_Slider_Y_valueChanged(int value);
  void on_Slider_Z_valueChanged(int value);
  void on_lineEd_X_returnPressed();
  void on_lineEd_Y_returnPressed();
  void on_lineEd_Z_returnPressed();
  void on_sendButton_clicked();
  void on_resetTopButton_clicked();
  void on_houseButton_clicked();
  void on_pushButton_clicked();
  void on_speedSlider_valueChanged(int value);
  void on_speedBox_returnPressed();
  void stopMotion();
  void on_stopButton_clicked();
  void on_gripopenButton_clicked();
  void on_gripcloseButton_clicked();
  void on_led_radius_returnPressed();
  void on_radioCircular_clicked();
  void on_radioCartCubic_clicked();
  void on_radioAngCubic_clicked();
  void on_radioLinear_clicked();
  void on_radioContinuous_clicked();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void updateJointState(float q0, float q1, float q2);

private:
	Ui::MainWindowDesign ui;
  void showValues();
	QNode qnode;
  Kinematics kinematics;
  DeltaPlanner deltaplanner;

};

}  // namespace delta_qtgui


#endif // delta_qtgui_MAIN_WINDOW_H
