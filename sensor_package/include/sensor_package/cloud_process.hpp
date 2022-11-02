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
#include <util/util_sensor.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/crop_box.h>
#include <tf2/utils.h>
#include <util/util_tf.hpp>

class CloudProcess
{
public:
    CloudProcess();
    pcl::PointCloud<PclXyz> planar_segmentar(pcl::PointCloud<PclXyz>);
    void set_crop_frame(std::string, std::string);
    pcl::PointCloud<PclXyz> cropbox_segmenter(pcl::PointCloud<PclXyz>);
    static pcl::PointCloud<PclXyz> downsample_by_voxelgrid(pcl::PointCloud<PclXyz>, double);
    static pcl::PointCloud<PclXyz> downsample_random(pcl::PointCloud<PclXyz>, int);

private:
    XmlRpc::XmlRpcValue param_list;
    ros::NodeHandle pnh_;
    geometry_msgs::Transform crop_trans_;
    void segment(pcl::PointIndices::Ptr, pcl::PointCloud<PclXyz>);
    pcl::PointCloud<PclXyz> extract(pcl::PointIndices::Ptr, pcl::PointCloud<PclXyz>);
};