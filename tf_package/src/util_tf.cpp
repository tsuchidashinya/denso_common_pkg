#include <util/util_tf.hpp>

UtilTF::UtilTF() :
    buffer_(),
    listener_(buffer_)
{

}

/**
 * @brief ROSのTFを取得する
 *
 * @param target target_frame
 * @param source source_frame
 * @return geometry_msgs::Transform
 */
geometry_msgs::Transform UtilTF::get_tf(std::string target, std::string source)
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
 * @brief xyzの指定した軸の方向に回転する
 *
 * @param rotated_quat クオータニオン型
 * @param xyz string型, "x" or "y" or "z"
 * @param angle
 * @return tf2::Quaternion
 */
tf2::Quaternion UtilTF::rotate_quaternion_by_axis(tf2::Quaternion rotated_quat, RotationOption option, double angle)
{
  tf2::Quaternion q_ori(0, 0, 0, 0);
  if (option == RotationOption::x)
  {
    q_ori.setX(1);
  }
  else if (option == RotationOption::y)
  {
    q_ori.setY(1);
  }
  else if (option == RotationOption::z)
  {
    q_ori.setZ(1);
  }
  tf2::Quaternion q_after, q_final;
  q_after = rotated_quat * q_ori * rotated_quat.inverse();
  tf2::Vector3 vec(q_after[0], q_after[1], q_after[2]);
  q_final.setRotation(vec, angle);
  return q_final;
}

tf2::Quaternion UtilTF::rotate_xyz_make(double x, double y, double z, tf2::Quaternion q_moto)
{
  tf2::Quaternion quaternion;
  quaternion = rotate_quaternion_by_axis(q_moto, RotationOption::x, x);
  quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::y, y);
  quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::z, z);
  return quaternion;
}

tf2::Quaternion UtilTF::rotate_xyz_make(double x, double y, double z)
{
  tf2::Quaternion quaternion(0, 0, 0, 1);
  quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::x, x);
  quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::y, y);
  quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::z, z);
  return quaternion;
}

void UtilTF::broadcast(geometry_msgs::TransformStamped transform)
{
    for (int i = 0; i < 3; i++) {
        br_.sendTransform(transform);
        ros::Duration(0.01).sleep();
    }
}

void UtilTF::static_broadcast(geometry_msgs::TransformStamped transform)
{
    for (int i = 0; i < 3; i++) {
        static_br_.sendTransform(transform);
        ros::Duration(0.01).sleep();
    }
}