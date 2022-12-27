#pragma once
#include <ros/ros.h>
#include <pcl_ros/impl/transforms.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include "mesh_sampler.hpp"
#include <tf/transform_datatypes.h>
#include <common_srvs/MeshCloudService.h>
#include <common_msgs/CloudData.h>
#include <common_msgs/PoseData.h>
#include <util/util.hpp>
#include <util/util_msg_data.hpp>
#include <tf_package/tf_function.hpp>

struct MeshOutType
{
    std::vector<common_msgs::CloudData> mesh_data;
    std::vector<common_msgs::PoseData> pose_data;
};
class MeshCloudServer
{
public:
    MeshCloudServer(ros::NodeHandle &);
    void set_parameter();
    pcl::PointCloud<PclXyz> create_mesh(std::string);
    pcl::PointCloud<PclXyz> transform_mesh(pcl::PointCloud<PclXyz>, std::string);
    bool service_callback(common_srvs::MeshCloudServiceRequest &, common_srvs::MeshCloudServiceResponse &);
    XmlRpc::XmlRpcValue param_list;
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::ServiceServer server_, visual_server_;
    ros::Timer timer_;
    std::string world_frame_, mesh_service_name_, sensor_frame_;
    std::vector<std::string> mesh_name_list_;
    std::vector<pcl::PointCloud<PclXyz>> mesh_stack_list_;
    int sample_points;
    TfFunction tf_func_;
    double LEAF_SIZE_;
};
