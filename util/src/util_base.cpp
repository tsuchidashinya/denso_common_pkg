/**
 * @file util_base.cpp
 * @author 土田真哉
 * @brief  UtilBaseクラスのソースファイル
 * @version 0.1
 * @date 2022-09-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <util/util_base.hpp>

/**
 * @brief 小数点以下に少しでも値があれば+1した整数値を渡す関数
 *
 * @param num
 * @return int
 */
int UtilBase::calcurate_round_up(double num)
{
  if (num - int(num))
  {
    return int(num) + 1;
  }
  else
  {
    return int(num);
  }
}

/**
 * @brief xyzの指定した軸の方向に回転する
 *
 * @param rotated_quat クオータニオン型
 * @param xyz string型, "x" or "y" or "z"
 * @param angle
 * @return tf2::Quaternion
 */
tf2::Quaternion UtilBase::rotate_quaternion_by_axis(tf2::Quaternion rotated_quat, std::string xyz, double angle)
{
  tf2::Quaternion q_ori(0, 0, 0, 0);
  if (xyz == "x")
  {
    q_ori.setX(1);
  }
  else if (xyz == "y")
  {
    q_ori.setY(1);
  }
  else if (xyz == "z")
  {
    q_ori.setZ(1);
  }
  else
  {
    throw std::runtime_error("rotate_quaternion_by_axis: x, y, z以外の文字が指定されています");
  }
  tf2::Quaternion q_after, q_final;
  q_after = rotated_quat * q_ori * rotated_quat.inverse();
  tf2::Vector3 vec(q_after[0], q_after[1], q_after[2]);
  q_final.setRotation(vec, angle);
  return q_final;
}

/**
 * @brief ディレクトリを生成
 *
 * @param path
 */
void UtilBase::mkdir(std::string path)
{
  if (!std::filesystem::exists(path))
  {
    std::filesystem::create_directories(path);
  }
}

/**
 * @brief pythonのos.path.joinと同じ機能を持つ、パスの結合関数
 *
 * @param str_front 結合されるパス
 * @param str_add 結合するパス
 * @return std::string 結合後のパス
 */
std::string UtilBase::join(std::string str_front, std::string str_add)
{
  if (str_front[str_front.size() - 1] == '/')
  {
    str_front = str_front.substr(0, str_front.size() - 1);
  }
  if (str_add[str_add.size() - 1] == '/')
  {
    str_add = str_add.substr(0, str_add.size() - 1);
  }
  if (str_add[0] == '/')
  {
    str_add = str_add.substr(1, str_add.size());
  }
  std::string output;
  output = str_front + "/" + str_add;
  return output;
}

/**
 * @brief 現在の時刻などをパスに表示するための関数
 *
 * @return std::string 月_日_時間_分_秒
 */
std::string UtilBase::get_time_str()
{
  time_t t_ = time(nullptr);
  const tm *localtime_ = localtime(&t_);
  std::string output = std::to_string(localtime_->tm_mon + 1) + "_" + std::to_string(localtime_->tm_mday) + "_" + std::to_string(localtime_->tm_hour) + "_" + std::to_string(localtime_->tm_min) + "_" + std::to_string(localtime_->tm_sec);
  return output;
}

/**
 * @brief ２つのベクトルの距離を返す
 *
 * @param vec1 ベクトル(配列の参照で渡してください 例: double vec1[3] = {0, 1, 0})
 * @param vec2  vec1と同様
 * @return double vec1とvec2のユークリッド距離
 */
double UtilBase::distance(double *vec1, double *vec2)
{
  int vec_size = sizeof(vec1) / sizeof(vec1[0]);
  double sum = 0;
  for (int i = 0; i < vec_size; i++)
  {
    sum += abs(vec1[i] - vec2[i]) * abs(vec1[i] - vec2[i]);
  }
  return sqrt(sum);
}

/**
 * @brief Transform型を簡単に作るための関数
 *
 * @param x
 * @param y
 * @param z
 * @param quaterion
 * @return geometry_msgs::Transform
 */
geometry_msgs::Transform UtilBase::geo_trans_make(double x, double y, double z, tf2::Quaternion quaterion)
{
  geometry_msgs::Transform output;
  output.translation.x = x;
  output.translation.y = y;
  output.translation.z = z;
  tf2::convert(quaterion, output.rotation);
  return output;
}

/**
 * @brief ROSのTFを取得する
 *
 * @param target target_frame
 * @param source source_frame
 * @return geometry_msgs::Transform
 */
geometry_msgs::Transform UtilBase::get_tf(std::string target, std::string source)
{
  geometry_msgs::TransformStamped final_tf;
  while (true)
  {
    try
    {
      final_tf = buffer_.lookupTransform(source, target, ros::Time(0));
      ROS_INFO_STREAM_ONCE("get_tf: source: " << source << "  target: " << target);
      break;
    }
    catch (const std::exception &e)
    {
      ROS_WARN_STREAM(e.what());
      ros::Duration(0.1).sleep();
      continue;
    }
  }
  return final_tf.transform;
}

/**
 * @brief geometry_msgs::Transform型からtf::StampedTransformへ変換する関数
 *
 * @param trans
 * @return tf::StampedTransform
 */
tf::StampedTransform UtilBase::make_stamped_trans(geometry_msgs::Transform trans)
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

/**
 * @brief int型のランダムな数値を生成する
 *
 * @param min 生成する数値の下限
 * @param max 生成する数値の上限
 * @return int
 */
int UtilBase::random_int(int min, int max)
{
  std::uniform_int_distribution<> distr(min, max);
  return distr(eng_);
}

/**
 * @brief float型のランダムな数値を生成する
 *
 * @param min 生成する数値の下限
 * @param max 生成する数値の上限
 * @return float
 */
float UtilBase::random_float(float min, float max)
{
  std::uniform_real_distribution<> distr(min, max);
  return distr(eng_);
}

std::string UtilBase::get_name_by_typeinfo(std::type_info const &iTypeInfo)
{
  char *aName;
  int status = 0;
  aName = abi::__cxa_demangle(iTypeInfo.name(), 0, 0, &status);
  std::string ret(aName);
  std::free(aName);
  return ret;
}
