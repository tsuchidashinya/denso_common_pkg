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
    set_parameter();
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
    
    crop_trans_ = tfbase_.tf_listen(target_frame, sensor_frame);
}

void CloudProcess::set_parameter()
{
    crop_x_min_ = param_list["x_min"];
    crop_y_min_ = param_list["y_min"];
    crop_z_min_ = param_list["z_min"];
    crop_x_max_ = param_list["x_max"];
    crop_y_max_ = param_list["y_max"];
    crop_z_max_ = param_list["z_max"];
}

