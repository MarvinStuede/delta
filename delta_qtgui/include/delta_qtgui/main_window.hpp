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
#include "delta_kinematics.h"
#include "delta_planner.hpp"
#include "delta_posereader.hpp"
#include "delta_wswatchdog.hpp"
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
  void setTeachedPoint(std::string pointname);
  void goPTP(const DeltaPlanner::InterpolationMode &mode);
  void goCircular();

  QNode qnode;
  //Thread/Instanz von Deltaplenner zur Bewegungskommandierung
  QThread* movThread;
  DeltaPlanner* deltaMov;

  Q_SIGNALS:
  void setMovFlag(const bool en);


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
  void on_homeButton_clicked();
  void on_speedSlider_valueChanged(int value);
  void on_speedBox_returnPressed();
  void stopMotion();
  void on_stopButton_clicked();
  void on_gripopenButton_clicked();
  void on_gripcloseButton_clicked();
  void on_led_radius_returnPressed();
  void on_radioCircular_clicked();
  void on_radioCartCubic_clicked();
  void on_radioLinear_clicked();
  void on_radioContinuous_clicked();
  void on_button_teachPoint_clicked();
  void setPoseListEntries();
  void on_led_pointName_textEdited(const QString &arg1);
  void on_listPoses_itemClicked(QListWidgetItem *item);
  void on_button_drawCross_clicked();
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void updateJointState(float q0, float q1, float q2);


private:
	Ui::MainWindowDesign ui;
  void showValues();
  Kinematics kinematics;
  DeltaPlanner deltaplanner;
  delta_posereader::PoseMap poses;
  WSWatchdog wswatchdog;

};

}  // namespace delta_qtgui


#endif // delta_qtgui_MAIN_WINDOW_H
