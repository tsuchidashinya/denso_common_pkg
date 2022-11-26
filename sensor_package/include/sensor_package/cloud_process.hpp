/**
 * @file cloud_process.hpp
 * @author tsuchidashinya (tsuchida.shinya413@mail.kyutech.jp)
 * @brief 
 * @version 0.1
 * @date 2022-09-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include <util/util_msg_data.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/crop_box.h>
#include <tf2/utils.h>
#include <tf_package/tf_basic.hpp>

struct MyPointType
{
    float x;
    float y;
    float z;
    int instance;
};

class CloudProcess
{
public:
    CloudProcess();
    pcl::PointCloud<PclXyz> planar_segmentar(pcl::PointCloud<PclXyz>);
    void set_crop_frame(std::string, std::string);
    pcl::PointCloud<PclXyz> cropbox_segmenter(pcl::PointCloud<PclXyz>);
    static pcl::PointCloud<PclXyz> downsample_by_voxelgrid(pcl::PointCloud<PclXyz>, double);
    template <typename T>
    static pcl::PointCloud<T> downsample_by_voxelgrid(pcl::PointCloud<T> pcl_data, double leaf_size)
    {
        pcl::PointCloud<T> output;
        pcl::VoxelGrid<T> voxelGrid;
        voxelGrid.setInputCloud(pcl_data.makeShared());
        voxelGrid.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxelGrid.filter(output);
        return output;
    }
    static pcl::PointCloud<PclXyz> downsample_random(pcl::PointCloud<PclXyz>, int);

private:
    void set_parameter();
    XmlRpc::XmlRpcValue param_list;
    ros::NodeHandle pnh_;
    geometry_msgs::Transform crop_trans_;
    void segment(pcl::PointIndices::Ptr, pcl::PointCloud<PclXyz>);
    pcl::PointCloud<PclXyz> extract(pcl::PointIndices::Ptr, pcl::PointCloud<PclXyz>);
    double crop_x_max_, crop_x_min_, crop_y_min_, crop_y_max_, crop_z_min_, crop_z_max_;
    TfBasic tfbase_;
};