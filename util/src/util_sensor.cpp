#include <util/util_sensor.hpp>

UtilSensor::UtilSensor()
    : pnh_("~"),
      param_list()
{
  pnh_.getParam("param_list", param_list);
}

/**
 * @brief pcl::PointCloud<pcl::PointXYZ>型からcommon_msgs::CloudData型へ変換する関数
 *
 * @param pcl_data
 * @return common_msgs::CloudData
 */
common_msgs::CloudData UtilSensor::pcl_to_cloudmsg(pcl::PointCloud<PclXyz> pcl_data)
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

/**
 * @brief common_msgs::CloudData型からpcl::PointCloud<pcl::PointXYZ>型へ変換する関数
 *
 * @param cloud_data
 * @return pcl::PointCloud<PclXyz>
 */
pcl::PointCloud<PclXyz> UtilSensor::cloudmsg_to_pcl(common_msgs::CloudData cloud_data)
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
 *        色の付け方で特別指定したい場合はutil_sensor_config.yamlに記述
 *
 * @param cloud_data
 * @return pcl::PointCloud<PclRgb>
 */
pcl::PointCloud<PclRgb> UtilSensor::cloudmsg_to_pclrgb(common_msgs::CloudData cloud_data)
{
  pcl::PointCloud<PclRgb> pcl_rgb_data;
  std::vector<int> ins_list;
  for (int i = 0; i < param_list["cloud_data"].size(); i++)
  {
    ins_list.push_back(param_list["cloud_data"][i]["instance"]);
  }
  for (int i = 0; i < cloud_data.x.size(); i++)
  {
    pcl_rgb_data.points[i].x = cloud_data.x[i];
    pcl_rgb_data.points[i].y = cloud_data.y[i];
    pcl_rgb_data.points[i].z = cloud_data.z[i];
    auto itr = std::find(ins_list.begin(), ins_list.end(), cloud_data.instance[i]);
    if (itr == ins_list.end())
    {
      pcl_rgb_data.points[i].r = util.random_int(0, 255);
      pcl_rgb_data.points[i].g = util.random_int(0, 255);
      pcl_rgb_data.points[i].b = util.random_int(0, 255);
    }
    else
    {
      const int index = std::distance(ins_list.begin(), itr);
      pcl_rgb_data.points[i].r = static_cast<int>(param_list["cloud_data"][i]["color"]["r"]);
      pcl_rgb_data.points[i].g = static_cast<int>(param_list["cloud_data"][i]["color"]["g"]);
      pcl_rgb_data.points[i].b = static_cast<int>(param_list["cloud_data"][i]["color"]["b"]);
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
sensor_msgs::PointCloud2 UtilSensor::cloudmsg_to_pc2(common_msgs::CloudData cloud_data)
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
sensor_msgs::PointCloud2 UtilSensor::cloudmsg_to_pc2_color(common_msgs::CloudData cloud_data)
{
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(cloudmsg_to_pclrgb(cloud_data), pc2);
  return pc2;
}

common_msgs::CloudData UtilSensor::pc2_to_cloudmsg(sensor_msgs::PointCloud2 pc2)
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
cv::Mat UtilSensor::img_to_cv(sensor_msgs::Image img_msg, std::string encording)
{

  cv_bridge::CvImageConstPtr cv_img_ptr;
  cv_img_ptr = cv_bridge::toCvCopy(img_msg, encording);
  cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
  cv_image = cv_img_ptr->image;
  return cv_image;
}