#pragma once
#include <common_msgs/CloudData.h>
#include <common_msgs/ObjectInfo.h>
#include <common_msgs/BoxPosition.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <util/util_msg_data.hpp>
#include <tf_package/tf_function.hpp>
struct ObjectTfNameType
{
    geometry_msgs::Transform trans;
    int index;
};

class SpaceHandlingLibrary
{
public:
    SpaceHandlingLibrary();
    static common_msgs::CloudData search_nearest_point(common_msgs::CloudData, common_msgs::CloudData, int, double);
    static common_msgs::CloudData search_nearest_point_on_unit(common_msgs::CloudData, common_msgs::CloudData, int, double);
    std::vector<common_msgs::CloudData> extract_occuluder(std::vector<common_msgs::CloudData>, double);
    std::vector<common_msgs::ObjectInfo> extract_occuluder(std::vector<common_msgs::ObjectInfo>, double);
private:
    std::vector<ObjectTfNameType> detect_occuluder(std::vector<std::string>, double);
    ros::NodeHandle pnh_;
    std::string world_frame_;
    XmlRpc::XmlRpcValue param_list;
    TfFunction tf_func_;
    void set_parameter();
};