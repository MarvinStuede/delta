/**
 * @file /include/delta_qtgui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef delta_qtgui_QNODE_HPP_
#define delta_qtgui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <rosgraph_msgs/Log.h>
#include <geometry_msgs/PoseStamped.h>
#include <delta_arduino/GetInfo.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delta_qtgui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
  void sendDeltaCmd(std::string cmd);
  void sendDeltaAngle(float t1, float t2, float t3,float v1, float v2, float v3);
  void rosoutCallback(const rosgraph_msgs::Log::ConstPtr &msg);
  std::string getDeltaInfo(std::string cmd);
  void getDeltaAngles(std::string cmd, float &t1, float &t2, float &t3);
  void sendDeltaKartPos(float x, float y, float z);
	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
  ros::Publisher cmdDelta;
  ros::Publisher cmdAngle;
  ros::Publisher cmdKart;
  ros::Subscriber rosout;
  delta_arduino::GetInfo infoSrv;
  ros::ServiceClient infoClient;

    QStringListModel logging_model;
};

}  // namespace delta_qtgui

#endif /* delta_qtgui_QNODE_HPP_ */
