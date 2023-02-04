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
#include <util_package/util_msg_data.hpp>

UtilMsgData::UtilMsgData()
    : pnh_("~"),
      param_list()
{
  set_parameter();
}

geometry_msgs::Vector3 UtilMsgData::vector3(float x, float y, float z)
{
  geometry_msgs::Vector3 vector3;
  vector3.x = x;
  vector3.y = y;
  vector3.z = z;
  return vector3;
}

geometry_msgs::Vector3 UtilMsgData::cloudmsg_to_vector3(common_msgs::CloudData cloud) {
  geometry_msgs::Vector3 vector3;
  int index = Util::random_int_static(0, cloud.x.size() - 1);
  vector3.x = cloud.x[index];
  vector3.y = cloud.y[index];
  vector3.z = cloud.z[index];
  return vector3;
}

common_msgs::CloudData UtilMsgData::pushback_cloud_point(common_msgs::CloudData cloud, int index, common_msgs::CloudData out_cloud)
{
  out_cloud.x.push_back(cloud.x[index]);
  out_cloud.y.push_back(cloud.y[index]);
  out_cloud.z.push_back(cloud.z[index]);
  out_cloud.instance.push_back(cloud.instance[index]);
  return out_cloud;
}

/*
1: sub_after
2: sub_before
*/
common_msgs::CloudData UtilMsgData::substitute_cloudmsg_para(common_msgs::CloudData sub_after, common_msgs::CloudData sub_before)
{
  sub_after.frame_id = sub_before.frame_id;
  sub_after.object_name = sub_before.object_name;
  sub_after.tf_name = sub_before.tf_name;
  return sub_after;
}

common_msgs::CloudData UtilMsgData::vector3_to_cloudmsg(geometry_msgs::Vector3 vector3)
{
  common_msgs::CloudData cloud;
  cloud.x.push_back(vector3.x);
  cloud.y.push_back(vector3.y);
  cloud.z.push_back(vector3.z);
  cloud.instance.push_back(0);
  return cloud;
}

common_msgs::PoseData UtilMsgData::stamped_to_pose(tf::StampedTransform tf_stamped)
{
    common_msgs::PoseData out_data;
    out_data.trans.x = tf_stamped.getOrigin().x();
    out_data.trans.y = tf_stamped.getOrigin().y();
    out_data.trans.z = tf_stamped.getOrigin().z();
    out_data.rot.x = tf_stamped.getRotation().x();
    out_data.rot.y = tf_stamped.getRotation().y();
    out_data.rot.z = tf_stamped.getRotation().z();
    out_data.rot.w = tf_stamped.getRotation().w();
    return out_data;
}

/**
 * @brief geometry_msgs::Transform型からtf::StampedTransformへ変換する関数
 *
 * @param trans
 * @return tf::StampedTransform
 */
tf::StampedTransform UtilMsgData::make_stamped_trans(geometry_msgs::Transform trans)
{
  tf::StampedTransform stamp_trans;
  tf::Vector3 vec3;
  vec3.setX(trans.translation.x);
  vec3.setY(trans.translation.y);
  vec3.setZ(trans.translation.z);
  stamp_trans.setOrigin(vec3);
  tf::Quaternion tq(trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w);
  stamp_trans.setRotation(tq);
  return stamp_trans;
}

common_msgs::CloudData UtilMsgData::draw_all_ins_cloudmsg(common_msgs::CloudData cloud, int instance)
{
    cloud.instance.resize(cloud.x.size());
    for (int i = 0; i < cloud.x.size(); i++) {
        cloud.instance[i] = instance;
    }
    return cloud;
}

std::vector<common_msgs::BoxPosition> UtilMsgData::set_classname_on_boxposition(std::vector<common_msgs::BoxPosition> box_position_list, std::string class_name)
{
    for (int i = 0; i < box_position_list.size(); i++) {
        box_position_list[i].object_name = class_name;
    }
    return box_position_list;
}

common_msgs::PoseData UtilMsgData::tranform_to_posedata(geometry_msgs::Transform trans)
{
  common_msgs::PoseData pose;
  pose.trans.x = trans.translation.x;
  pose.trans.y = trans.translation.y;
  pose.trans.z = trans.translation.z;
  pose.rot.x = trans.rotation.x;
  pose.rot.y = trans.rotation.y;
  pose.rot.z = trans.rotation.z;
  pose.rot.w = trans.rotation.w;
  return pose;
}

geometry_msgs::Transform UtilMsgData::posedata_to_transform(common_msgs::PoseData pose)
{
  geometry_msgs::Transform trans;
  trans.translation.x = pose.trans.x;
  trans.translation.y = pose.trans.y;
  trans.translation.z = pose.trans.z;
  trans.rotation.x = pose.rot.x;
  trans.rotation.y = pose.rot.y;
  trans.rotation.z = pose.rot.z;
  trans.rotation.w = pose.rot.w;
  return trans;
}

std::map<int, int> UtilMsgData::get_instance_dict(common_msgs::CloudData cloud)
{
  std::map<int, int> info;
  std::vector<int> instance_list;
  for (int i = 0; i < cloud.x.size(); i++) {
    int same_exist = Util::find_element_vector(instance_list, cloud.instance[i]);
    if (same_exist == -1) {
      instance_list.push_back(cloud.instance[i]);
      info[cloud.instance[i]] = 0;
    }
    info[cloud.instance[i]]++;
  }
  return info;
}

std::vector<float> UtilMsgData::caminfo_to_floatlist(sensor_msgs::CameraInfo cinfo)
{
  std::vector<float> K;
  for (int i = 0; i < cinfo.K.size(); i++) {
    K.push_back(cinfo.K[i]);
  }
  return K;
}

/*
1: cloud_data
2: ins_before 変更対象のインスタンス番号
3: ins_after 変更後のインスタンス番号
*/
common_msgs::CloudData UtilMsgData::change_ins_cloudmsg(common_msgs::CloudData cloud, int ins_before, int ins_after)
{
  common_msgs::CloudData outdata;
  for (int i = 0; i < cloud.x.size(); i++) {
    outdata.x.push_back(cloud.x[i]);
    outdata.y.push_back(cloud.y[i]);
    outdata.z.push_back(cloud.z[i]);
    if (cloud.instance[i] == ins_before) {
      outdata.instance.push_back(ins_after);
    }
    else {
      outdata.instance.push_back(cloud.instance[i]);
    }
  }
  outdata = substitute_cloudmsg_para(outdata, cloud);
  return outdata;
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
  outdata = substitute_cloudmsg_para(outdata, cloud);
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
  outdata.tf_name = cloud.tf_name;
  outdata.object_name = cloud.object_name;
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
  outdata.object_name = cloud_ori.object_name + cloud_add.object_name;
  outdata.tf_name = cloud_ori.tf_name + cloud_add.tf_name;
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


bool UtilMsgData::cloud_size_ok(common_msgs::CloudData cloud)
{
  if (cloud.x.size() == cloud.y.size() && cloud.x.size() == cloud.z.size() && cloud.x.size() == cloud.instance.size()) {
    return true;
  }
  else {
    return false;
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

pcl::PointCloud<pcl::PointXYZL> UtilMsgData::cloudmsg_to_pclLabel(common_msgs::CloudData cloud_data)
{
  pcl::PointCloud<pcl::PointXYZL> mypoints;
  for (int i = 0; i < cloud_data.x.size(); i++) {
    pcl::PointXYZL mypoint;
    mypoint.x = cloud_data.x[i];
    mypoint.y = cloud_data.y[i];
    mypoint.z = cloud_data.z[i];
    mypoint.label = cloud_data.instance[i];
    mypoints.push_back(mypoint);
  }
  mypoints.header.frame_id = cloud_data.tf_name;
  return mypoints;
}

common_msgs::CloudData UtilMsgData::pclLabel_to_cloudmsg(pcl::PointCloud<pcl::PointXYZL> mypoints)
{
  common_msgs::CloudData cloud_data;
  for (int i = 0; i < mypoints.points.size(); i++) {
    cloud_data.x.push_back(mypoints.points[i].x);
    cloud_data.y.push_back(mypoints.points[i].y);
    cloud_data.z.push_back(mypoints.points[i].z);
    cloud_data.instance.push_back(mypoints.points[i].label);
  }
  cloud_data.tf_name = mypoints.header.frame_id;
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
  outdata.object_name = boxes.object_name;
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
  boxes.object_name = yolo_boxes.object_name;
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
    // ROS_INFO_STREAM(object_info.tf_name << "  x: " << object_info.position.translation.x << "  y: " << object_info.position.translation.y << "   z: " << object_info.position.translation.z);
    model.model_name = object_info.tf_name;
    model.pose.position.x = object_info.position.translation.x;
    model.pose.position.y = object_info.position.translation.y;
    model.pose.position.z = object_info.position.translation.z;
    tf2::convert(object_info.position.rotation, model.pose.orientation);
    return model;
}