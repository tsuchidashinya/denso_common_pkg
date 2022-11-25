/**
 * @file util_msg_data.cpp
 * @author tsuchidashinya (tsuchida.shinya413@mail.kyutech.jp)
 * @brief
 * @version 0.1
 * @date 2022-09-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <util/util_msg_data.hpp>

UtilMsgData::UtilMsgData()
    : pnh_("~"),
      param_list()
{
  set_parameter();
}

std::vector<float> UtilMsgData::caminfo_to_floatlist(sensor_msgs::CameraInfo cinfo)
{
  std::vector<float> K;
  for (int i = 0; i < cinfo.K.size(); i++) {
    K.push_back(cinfo.K[i]);
  }
  return K;
}

common_msgs::CloudData UtilMsgData::extract_ins_cloudmsg(common_msgs::CloudData cloud, int ins)
{
  common_msgs::CloudData outdata;
  for (int i = 0; i < cloud.x.size(); i++) {
    if (cloud.instance[i] == ins) {
      outdata.x.push_back(cloud.x[i]);
      outdata.y.push_back(cloud.y[i]);
      outdata.z.push_back(cloud.z[i]);
      outdata.instance.push_back(cloud.instance[i]);
    }
  }
  outdata.cloud_name = cloud.cloud_name;
  return outdata;
}

common_msgs::CloudData UtilMsgData::remove_ins_cloudmsg(common_msgs::CloudData cloud, int remove_ins)
{
  common_msgs::CloudData outdata;
  for (int i = 0; i < cloud.x.size(); i++) {
    if (cloud.instance[i] != remove_ins) {
      outdata.x.push_back(cloud.x[i]);
      outdata.y.push_back(cloud.y[i]);
      outdata.z.push_back(cloud.z[i]);
      outdata.instance.push_back(cloud.instance[i]);
    }
  }
  outdata.cloud_name = cloud.cloud_name;
  return outdata;
}

common_msgs::CloudData UtilMsgData::concat_cloudmsg(common_msgs::CloudData cloud_ori, common_msgs::CloudData cloud_add)
{
  common_msgs::CloudData outdata;
  for (int i = 0; i < cloud_ori.x.size(); i++) {
    outdata.x.push_back(cloud_ori.x[i]);
    outdata.y.push_back(cloud_ori.y[i]);
    outdata.z.push_back(cloud_ori.z[i]);
    outdata.instance.push_back(cloud_ori.instance[i]);
  }
  for (int i = 0; i < cloud_add.x.size(); i++) {
    outdata.x.push_back(cloud_add.x[i]);
    outdata.y.push_back(cloud_add.y[i]);
    outdata.z.push_back(cloud_add.z[i]);
    outdata.instance.push_back(cloud_add.instance[i]);
  }
  outdata.cloud_name = cloud_ori.cloud_name + cloud_add.cloud_name;
  return outdata;
}

void UtilMsgData::set_parameter()
{
  pnh_.getParam("common_parameter/util_msg_data", param_list);
}

sensor_msgs::PointCloud2 UtilMsgData::pclrgb_to_pc2_color(pcl::PointCloud<PclRgb> pclrgb_data)
{
  sensor_msgs::PointCloud2 sensor_data;
  pcl::toROSMsg(pclrgb_data, sensor_data);
  return sensor_data;
}

pcl::PointCloud<PclRgb> UtilMsgData::pc2_color_to_pclrgb(sensor_msgs::PointCloud2 sensor_data)
{
  pcl::PointCloud<PclRgb> pclrbg_data;
  pcl::fromROSMsg(sensor_data, pclrbg_data);
  return pclrbg_data;
}


void UtilMsgData::cloud_size_ok(common_msgs::CloudData &cloud)
{
  if (cloud.x.size() == cloud.y.size() == cloud.z.size() == cloud.instance.size()) {
    ;
  }
  else {
    cloud.instance.resize(cloud.x.size());
  }
}

/**
 * @brief pcl::PointCloud<pcl::PointXYZ>型からcommon_msgs::CloudData型へ変換する関数
 *
 * @param pcl_data
 * @return common_msgs::CloudData
 */
common_msgs::CloudData UtilMsgData::pcl_to_cloudmsg(pcl::PointCloud<PclXyz> pcl_data)
{
  common_msgs::CloudData cloud_data;
  cloud_data.x.resize(pcl_data.points.size());
  cloud_data.y.resize(pcl_data.points.size());
  cloud_data.z.resize(pcl_data.points.size());
  cloud_data.instance.resize(pcl_data.points.size());
  for (int i = 0; i < pcl_data.points.size(); i++)
  {
    cloud_data.x[i] = pcl_data.points[i].x;
    cloud_data.y[i] = pcl_data.points[i].y;
    cloud_data.z[i] = pcl_data.points[i].z;
    cloud_data.instance[i] = 0;
  }
  return cloud_data;
}

sensor_msgs::PointCloud2 UtilMsgData::pcl_to_pc2(pcl::PointCloud<PclXyz> pcl_data)
{
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(pcl_data, pc2);
  return pc2;
}

pcl::PointCloud<PclXyz> UtilMsgData::pc2_to_pcl(sensor_msgs::PointCloud2 pc2)
{
  pcl::PointCloud<PclXyz> pcl_data;
  pcl::fromROSMsg(pc2, pcl_data);
  return pcl_data;
}

/**
 * @brief common_msgs::CloudData型からpcl::PointCloud<pcl::PointXYZ>型へ変換する関数
 *
 * @param cloud_data
 * @return pcl::PointCloud<PclXyz>
 */
pcl::PointCloud<PclXyz> UtilMsgData::cloudmsg_to_pcl(common_msgs::CloudData cloud_data)
{
  pcl::PointCloud<PclXyz> pcl_data;
  pcl_data.points.resize(cloud_data.x.size());
  for (int i = 0; i < cloud_data.x.size(); i++)
  {
    pcl_data.points[i].x = cloud_data.x[i];
    pcl_data.points[i].y = cloud_data.y[i];
    pcl_data.points[i].z = cloud_data.z[i];
  }
  return pcl_data;
}

/**
 * @brief common_msgs::CloudData型からpcl::PointCloud<pcl::PointXYZRGB>型へ変換する関数
 *        色の付け方で特別指定したい場合はutil_msg_data_config.yamlに記述
 *
 * @param cloud_data
 * @return pcl::PointCloud<PclRgb>
 */
pcl::PointCloud<PclRgb> UtilMsgData::cloudmsg_to_pclrgb(common_msgs::CloudData cloud_data)
{
  pcl::PointCloud<PclRgb> pcl_rgb_data;
  pcl_rgb_data.points.resize(cloud_data.x.size());
  std::vector<int> ins_list;
  int random_red = util.random_int(0, 255);
  int random_blue = util.random_int(0, 255);
  int random_green = util.random_int(0, 255);
  for (int i = 0; i < param_list.size(); i++)
  {
    ins_list.push_back(param_list[i]["instance"]);
  }
  for (int i = 0; i < cloud_data.x.size(); i++)
  {
    pcl_rgb_data.points[i].x = cloud_data.x[i];
    pcl_rgb_data.points[i].y = cloud_data.y[i];
    pcl_rgb_data.points[i].z = cloud_data.z[i];
    auto itr = std::find(ins_list.begin(), ins_list.end(), cloud_data.instance[i]);
    if (itr == ins_list.end())
    {
      pcl_rgb_data.points[i].r = random_red;
      pcl_rgb_data.points[i].g = random_green;
      pcl_rgb_data.points[i].b = random_blue;
    }
    else
    {
      const int index = std::distance(ins_list.begin(), itr);
      pcl_rgb_data.points[i].r = static_cast<int>(param_list[index]["color"][0]);
      pcl_rgb_data.points[i].g = static_cast<int>(param_list[index]["color"][1]);
      pcl_rgb_data.points[i].b = static_cast<int>(param_list[index]["color"][2]);
    }
  }
  return pcl_rgb_data;
}

/**
 * @brief common_msgs::CloudData型からsensor_msgs::PointCloud2型に変換
 *
 * @param cloud_data
 * @return sensor_msgs::PointCloud2
 */
sensor_msgs::PointCloud2 UtilMsgData::cloudmsg_to_pc2(common_msgs::CloudData cloud_data)
{
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(cloudmsg_to_pcl(cloud_data), pc2);
  return pc2;
}

/**
 * @brief common_msgs::CloudData型からsensor_msgs::PointCloud2型に色付きで変換
 *
 * @param cloud_data
 * @return sensor_msgs::PointCloud2
 */
sensor_msgs::PointCloud2 UtilMsgData::cloudmsg_to_pc2_color(common_msgs::CloudData cloud_data)
{
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(cloudmsg_to_pclrgb(cloud_data), pc2);
  return pc2;
}

common_msgs::CloudData UtilMsgData::pc2_to_cloudmsg(sensor_msgs::PointCloud2 pc2)
{
  common_msgs::CloudData cloud_data;
  pcl::PointCloud<PclXyz> pcl_data;
  pcl::fromROSMsg(pc2, pcl_data);
  cloud_data = pcl_to_cloudmsg(pcl_data);
  return cloud_data;
}

/**
 * @brief sensorイメージからOpenCVのcv::Mat型に変換します。
 *
 * @param img_msg sensor_msgs::Image
 * @param encording エンコーディングを入力します。基本的には
 *
 * カラーであればsensor_msgs::image_encodings::BGR8であり,　
 *                  　　
 * 白黒であればsensor_msgs::image_encodings::MONO8が適切です。
 * @return cv::Mat
 */
cv::Mat UtilMsgData::rosimg_to_cvimg(sensor_msgs::Image img_msg, std::string encording)
{

  cv_bridge::CvImageConstPtr cv_img_ptr;
  cv_img_ptr = cv_bridge::toCvCopy(img_msg, encording);
  cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
  cv_image = cv_img_ptr->image;
  return cv_image;
}

sensor_msgs::Image UtilMsgData::cvimg_to_rosimg(cv::Mat img, std::string encording)
{
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), encording, img).toImageMsg();
  return *msg;
}

YoloFormat UtilMsgData::pascalvoc_to_yolo(common_msgs::BoxPosition boxes, ImageSize image_size)
{
  YoloFormat outdata;
  boxes = box_position_normalized(boxes);
  outdata.x = (boxes.x_one + boxes.x_two) / (2 * image_size.width);
  outdata.y = (boxes.y_one + boxes.y_two) / (2 * image_size.height);
  outdata.w = (boxes.x_two - boxes.x_one) / image_size.width;
  outdata.h = (boxes.y_two - boxes.y_one) / image_size.height;
  outdata.tf_name = boxes.tf_name;
  outdata.object_class_name = boxes.object_class_name;
  return outdata;
}

common_msgs::BoxPosition UtilMsgData::box_position_normalized(common_msgs::BoxPosition boxes)
{
  if (boxes.x_one > boxes.x_two) {
    std::swap(boxes.x_one, boxes.x_two);
  }
  if (boxes.y_one > boxes.y_two) {
    std::swap(boxes.y_one, boxes.y_two);
  }
  return boxes;
}

common_msgs::BoxPosition UtilMsgData::yolo_to_pascalvoc(YoloFormat yolo_boxes, ImageSize image_size)
{
  common_msgs::BoxPosition boxes;
  boxes.x_one = (yolo_boxes.x * 2 - yolo_boxes.w) * image_size.width / 2;
  boxes.x_two = (yolo_boxes.x * 2 + yolo_boxes.w) * image_size.width / 2;
  boxes.y_one = (yolo_boxes.y * 2 - yolo_boxes.h) * image_size.height / 2;
  boxes.y_two = (yolo_boxes.y * 2 + yolo_boxes.h) * image_size.height / 2;
  boxes.object_class_name = yolo_boxes.object_class_name;
  boxes.tf_name = yolo_boxes.tf_name;
  return boxes;
}

/**
 * @brief gazebo_modelメッセージを生成する関数です。
 *
 * @param object_name Gazeboオブジェクトの名前
 * @param trans geometry_msgs::Transform型の姿勢データ
 * @return gazebo_msgs::ModelState
 */
gazebo_msgs::ModelState UtilMsgData::make_gazebo_model_state(std::string object_name, geometry_msgs::Transform trans)
{
    gazebo_msgs::ModelState model;
    model.model_name = object_name;
    model.pose.position.x = trans.translation.x;
    model.pose.position.y = trans.translation.y;
    model.pose.position.z = trans.translation.z;
    tf2::convert(trans.rotation, model.pose.orientation);
    return model;
}

gazebo_msgs::ModelState UtilMsgData::make_gazebo_model_state(common_msgs::ObjectInfo object_info)
{
    gazebo_msgs::ModelState model;
    model.model_name = object_info.tf_name;
    model.pose.position.x = object_info.position.translation.x;
    model.pose.position.y = object_info.position.translation.y;
    model.pose.position.z = object_info.position.translation.z;
    tf2::convert(object_info.position.rotation, model.pose.orientation);
    return model;
}