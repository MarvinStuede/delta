
#ifndef DELTA_POSEREADER_H
#define DELTA_POSEREADER_H

#include <ros/package.h>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#endif
#include <geometry_msgs/Point.h>

namespace delta_posereader
{

typedef std::map<std::string, geometry_msgs::Point> PoseMap;

PoseMap read(std::string filename = "poses.yaml");
void write(delta_posereader::PoseMap poses,std::string filename = "poses.yaml");

}

#endif // DELTA_POSEREADER_H
