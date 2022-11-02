/**
 * @file cloud_process.cpp
 * @author tsuchidashinya (tsuchida.shinya413@mail.kyutech.jp)
 * @brief
 * @version 0.1
 * @date 2022-09-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <sensor_package/cloud_process.hpp>

CloudProcess::CloudProcess()
    : pnh_("~")
{
    pnh_.getParam("cloud_process", param_list);
}

/**
 * @brief 切り分けるべき点群を探索し、第一引数のinlinerに情報を格納
 *
 * @param inliner　セグメンテーションの情報が格納される
 * @param pcl_data　入力点群
 */
void CloudProcess::segment(pcl::PointIndices::Ptr inliner, pcl::PointCloud<PclXyz> pcl_data)
{
    pcl::ModelCoefficients coefficients;
    pcl::SACSegmentation<PclXyz> segment;
    segment.setModelType(pcl::SACMODEL_PLANE);
    segment.setMethodType(pcl::SAC_RANSAC);
    segment.setMaxIterations(1000);
    segment.setDistanceThreshold(param_list["planar_threshold"]);
    segment.setInputCloud(pcl_data.makeShared());
    segment.segment(*inliner, coefficients);
}

/**
 * @brief inlierからの情報をもとに、実際に点群の抽出する
 *
 * @param inlier　前のsegment関数で格納された変数をそのまま入力
 * @param pcl_data 入力点群
 * @return pcl::PointCloud<PclXyz> 出力点群
 */
pcl::PointCloud<PclXyz> CloudProcess::extract(pcl::PointIndices::Ptr inlier, pcl::PointCloud<PclXyz> pcl_data)
{
    pcl::PointCloud<PclXyz> segmented_pcl, without_segmented_pcl;
    pcl::ExtractIndices<PclXyz> extract;
    extract.setInputCloud(pcl_data.makeShared());
    extract.setIndices(inlier);
    extract.setNegative(true);
    extract.filter(without_segmented_pcl);
    return without_segmented_pcl;
}

/**
 * @brief 平面の部分を除去して点群を軽くする
 *
 * @param pcl_data 入力点群
 * @return pcl::PointCloud<PclXyz> 出力点群
 */
pcl::PointCloud<PclXyz> CloudProcess::planar_segmentar(pcl::PointCloud<PclXyz> pcl_data)
{
    pcl::PointIndices::Ptr inliner(new pcl::PointIndices());
    segment(inliner, pcl_data);
    pcl_data = extract(inliner, pcl_data);
    return pcl_data;
}

/**
 * @brief crop_box関数で範囲限定して点群を抽出する際に、どの点(TF)基準の範囲なのかを決定する
 *
 * @param sensor_frame センサーのTF名
 * @param target_frame 範囲限定する基準のTF
 */
void CloudProcess::set_crop_frame(std::string sensor_frame, std::string target_frame)
{
    UtilBase util;
    crop_trans_ = util.get_tf(target_frame, sensor_frame);
}

/**
 * @brief 範囲限定して点群を抽出する関数
 * ※　先にset_crop_frame関数を実行して抽出する基準のTFをセッティングしてください
 *
 * @param pcl_data 入力点群
 * @return pcl::PointCloud<PclXyz> 出力点群
 */
pcl::PointCloud<PclXyz> CloudProcess::cropbox_segmenter(pcl::PointCloud<PclXyz> pcl_data)
{
    Eigen::Vector4f minPoint, maxPoint;
    minPoint[0] = std::stof(param_list["crop_box"]["x_min"]);
    minPoint[1] = std::stof(param_list["crop_box"]["y_min"]);
    minPoint[2] = std::stof(param_list["crop_box"]["z_min"]);
    maxPoint[0] = std::stof(param_list["crop_box"]["x_max"]);
    maxPoint[1] = std::stof(param_list["crop_box"]["y_max"]);
    maxPoint[2] = std::stof(param_list["crop_box"]["z_max"]);

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

    pcl::CropBox<PclXyz> cropBox;
    cropBox.setInputCloud(pcl_data.makeShared());
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setTranslation(boxTranslation);
    cropBox.setRotation(boxRotation);

    pcl::PointCloud<PclXyz> output;
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
pcl::PointCloud<PclXyz> CloudProcess::downsample_by_voxelgrid(pcl::PointCloud<PclXyz> pcl_data, double leaf_size)
{
    pcl::PointCloud<PclXyz> output;
    pcl::VoxelGrid<PclXyz> voxelGrid;
    voxelGrid.setInputCloud(pcl_data.makeShared());
    voxelGrid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxelGrid.filter(output);
    return output;
}

/**
 * @brief ランダムダウンサンプリングを行う関数
 *
 * @param pcl_data 入力点群
 * @param resolution 抽出したい点群数
 * @return pcl::PointCloud<PclXyz> 出力点群
 */
pcl::PointCloud<PclXyz> CloudProcess::downsample_random(pcl::PointCloud<PclXyz> pcl_data, int resolution)
{
    pcl::PointCloud<PclXyz> output;
    pcl::RandomSample<PclXyz> randamSampling;
    randamSampling.setInputCloud(pcl_data.makeShared());
    randamSampling.setSample(resolution);
    randamSampling.filter(output);
    return output;
}