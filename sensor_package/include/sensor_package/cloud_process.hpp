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
#include <tf_package/tf_function.hpp>


class CloudProcess
{
public:
    CloudProcess();
    pcl::PointCloud<PclXyz> planar_segmentar(pcl::PointCloud<PclXyz>);
    void set_crop_frame(std::string, std::string);

    /**
     * @brief 範囲限定して点群を抽出する関数
     * ※　先にset_crop_frame関数を実行して抽出する基準のTFをセッティングしてください
     *
     * @param pcl_data 入力点群
     * @return pcl::PointCloud<PclXyz> 出力点群
     */
    template <typename T>
    pcl::PointCloud<T> cropbox_segmenter(pcl::PointCloud<T> pcl_data)
    {
        Eigen::Vector4f minPoint, maxPoint;
        minPoint[0] = crop_x_min_;
        minPoint[1] = crop_y_min_;
        minPoint[2] = crop_z_min_;
        maxPoint[0] = crop_x_max_;
        maxPoint[1] = crop_y_max_;
        maxPoint[2] = crop_z_max_;

        Eigen::Vector3f boxTranslation, boxRotation;
        boxTranslation[0] = crop_trans_.translation.x;
        boxTranslation[1] = crop_trans_.translation.y;
        boxTranslation[2] = crop_trans_.translation.z;
        double roll, pitch, yaw;
        tf2::Quaternion quat;
        tf2::convert(crop_trans_.rotation, quat);
        tf2::getEulerYPR(quat, yaw, pitch, roll);
        boxRotation[0] = roll;
        boxRotation[1] = pitch;
        boxRotation[2] = yaw;

        pcl::CropBox<T> cropBox;
        cropBox.setInputCloud(pcl_data.makeShared());
        cropBox.setMin(minPoint);
        cropBox.setMax(maxPoint);
        cropBox.setTranslation(boxTranslation);
        cropBox.setRotation(boxRotation);

        pcl::PointCloud<T> output;
        cropBox.filter(output);
        return output;
    }

    /**
     * @brief ボクセルグリッドフィルターを用いてダウンサンプリングする関数
     *
     * @param pcl_data 入力点群
     * @param leaf_size 0.003あたりが妥当です。
     * @return pcl::PointCloud<PclXyz> 出力点群
     */
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
    TfFunction tfbase_;
};