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
  void angleGetAndSend();
  void getCubicAngle(float qs, float qz, float te, float t, float ve,float &q, float &qd);
  void goCubic(float te,float stepSize);
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
  void on_contsendCheck_clicked(bool checked);
  void on_cubicCheck_clicked(bool checked);
  void on_sliderAngleStep_valueChanged(int value);
      void on_resetTopButton_clicked();
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically




private:
	Ui::MainWindowDesign ui;
  void showValues();
	QNode qnode;
  Kinematics kinematics;
  DeltaPlanner deltaplanner;
};

}  // namespace delta_qtgui


#endif // delta_qtgui_MAIN_WINDOW_H
