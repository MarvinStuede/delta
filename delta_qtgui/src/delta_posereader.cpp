#include "delta_posereader.hpp"
#include <fstream>

delta_posereader::PoseMap delta_posereader::read(std::string filename)
{
    delta_posereader::PoseMap pose_map;

    // if path is relative: use default directory
    if(filename[0] != '/')
    {

        std::string path = ros::package::getPath("launcher");
        path.append("/cfg/");
        path.append(filename);
        filename = path;
    }

    std::vector<YAML::Node> nodes;
    try
    {
         nodes = YAML::LoadAllFromFile(filename);
    }
    catch(YAML::BadFile &ex)
    {
        ROS_ERROR("%s", ex.what());
        ROS_ERROR("Failed to load poses from file %s", filename.c_str());
        return pose_map;
    }

    for(uint i=0; i<nodes.size();i++)
    {
        std::string name = nodes[i].begin()->first.as<std::string>();

        geometry_msgs::Point pose;
        YAML::Node EE_poses = nodes[i].begin()->second;

        for(YAML::const_iterator it=EE_poses.begin();it!=EE_poses.end();++it)
        {
            YAML::const_iterator map_it = it->begin();
            std::string coordinate_name = map_it->first.as<std::string>();
            double coordinate_value = map_it->second.as<double>();


            if(!coordinate_name.compare("x"))
                pose.x = coordinate_value;
            else if(!coordinate_name.compare("y"))
                pose.y = coordinate_value;
            else if(!coordinate_name.compare("z"))
                pose.z = coordinate_value;
        }

        pose_map[name] = pose;
    }

    return pose_map;
}
void delta_posereader::write(delta_posereader::PoseMap poses,std::string filename){
    if(filename[0] != '/')
    {

        std::string path = ros::package::getPath("launcher");
        path.append("/cfg/");
        path.append(filename);
        filename = path;
    }

    std::cout << "Saving to " << filename<< std::endl;
    std::ofstream fout(filename.c_str());

    for(PoseMap::const_iterator it=poses.begin(); it != poses.end(); ++it)
    {
        fout << "---" << std::endl;

        geometry_msgs::Point pose = it->second;


        YAML::Node seq_node;
        seq_node[0]["x"] = pose.x;
        seq_node[1]["y"] = pose.y;
        seq_node[2]["z"] = pose.z;

        YAML::Node map_node;
        map_node[it->first] = seq_node;

        fout << map_node << std::endl;
    }

}
