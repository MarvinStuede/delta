/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <delta_arduino/cmdAngle.h>
#include <sstream>
#include "../include/delta_qtgui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delta_qtgui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}
void QNode::rosoutCallback(const rosgraph_msgs::Log::ConstPtr &msg){
  //log(msg->level,msg->msg);
int i = (unsigned char)msg->level;

std::stringstream ss;
ss << msg->msg;

 std::string s =ss.str();
ROS_INFO(s.c_str());
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"delta_qtgui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  cmdDelta = n.advertise<std_msgs::String>("/delta/command",1000);
  cmdAngle = n.advertise<delta_arduino::cmdAngle>("/delta/set_angle",1000);
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
 // rosout = n.subscribe("rosout_agg",1000,&QNode::rosoutCallback,this);
  infoClient = n.serviceClient<delta_arduino::GetInfo>("delta/get_info");

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"delta_qtgui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  cmdDelta = n.advertise<std_msgs::String>("/delta/command",1000);
  cmdAngle = n.advertise<delta_arduino::cmdAngle>("/delta/set_angle",1000);
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
   infoClient = n.serviceClient<delta_arduino::GetInfo>("delta/get_info");
  //rosout = n.subscribe("rosout_agg",1000,&QNode::rosoutCallback,this);
	start();
	return true;
}

void QNode::sendDeltaCmd(std::string cmd){
  if(ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << cmd;
    msg.data = ss.str();
    cmdDelta.publish(msg);
    log(Info,std::string("I sent: ")+msg.data);
  }
}
std::string QNode::getDeltaInfo(std::string cmd){
    infoSrv.request.in=cmd;
    infoClient.call(infoSrv);

    log(Info,std::string("Received from Info Call: ")+infoSrv.response.out);
    return infoSrv.response.out;
}
void QNode::getDeltaAngles(std::string cmd, float &t1, float &t2, float &t3){
  infoSrv.request.in=cmd;
  infoClient.call(infoSrv);
  t1=infoSrv.response.theta1;
  t2=infoSrv.response.theta2;
  t3=infoSrv.response.theta3;

}

void QNode::sendDeltaAngle(float t1, float t2, float t3, float v1, float v2, float v3){
  if(ros::ok()) {
    delta_arduino::cmdAngle angles;
    angles.theta1=t1;
    angles.theta2=t2;
    angles.theta3=t3;
    angles.vtheta1=v1;
    angles.vtheta2=v2;
    angles.vtheta3=v3;
    cmdAngle.publish(angles);

  }
}

void QNode::run() {
	ros::Rate loop_rate(1);

	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();

	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace delta_qtgui
