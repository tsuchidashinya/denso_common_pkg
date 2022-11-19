#include <tf_package/tf_basic.hpp>

TfBasic::TfBasic() :
    buffer_(),
    listener_(buffer_)
{

}

/*
1: double: xyz_step
2: double: qxyz_step
*/
KeyBoardTf TfBasic::get_keyboard_tf(double xyz_step, double qxyz_step)
{
    KeyBoardTf output;
    output.quit = false;
    geometry_msgs::Transform final_change_trans;
    final_change_trans.translation.x = 0;
    final_change_trans.translation.y = 0;
    final_change_trans.translation.z = 0;
    tf2::Quaternion q(0, 0, 0, 1);
    tf2::convert(q, final_change_trans.rotation);
    ROS_INFO_STREAM("1:x     2:y    3:z   4:q_x    5:q_y     6:q_z ");
    ROS_INFO_STREAM("n: -x   m: +x   j: -y   k: +y  u: -z    i: +z");
    ROS_INFO_STREAM("c: -qx   v: +qx   d: -qy   f: +qy   e: -qz    r: +qz");
    ROS_INFO_STREAM("aq: decide location and quit");
    int para_index;
    double value;
    std::vector<std::string> v;
    std::string str1, s;
    while (str1 == "") {
        std::getline(std::cin, str1);
    }
    
    std::stringstream ss{str1};
    while (std::getline(ss, s, ' ')) {
        v.push_back(s);
    }
    if (v[0] == "1") {
        // std::cout << v[0] << "   ";
        if (v.size() == 1) {
            std::cout << "error" << std::endl;
        }
        else {
            try {
                final_change_trans.translation.x = std::stod(v[1]);
            }
            catch (...) {
                ROS_ERROR_STREAM("error");
            }
        } 
    }
    else if (v[0] == "2") {
        if (v.size() == 1) {
            std::cout << "error" << std::endl;
        }
        else {
            try {
                final_change_trans.translation.y = std::stod(v[1]);
            }
            catch (...) {
                ROS_ERROR_STREAM("error");
            }
        }
    }
    else if (v[0] == "3") {
        if (v.size() == 1) {
            std::cout << "error" << std::endl;
        }
        else {
            try {
                final_change_trans.translation.z = std::stod(v[1]);
            }
            catch (...) {
                ROS_ERROR_STREAM("error");
            }
        }
    }
    else if (v[0] == "4") {
        if (v.size() == 1) {
            std::cout << "error" << std::endl;
        }
        else {
            try {
                final_change_trans.rotation.x = std::stod(v[1]);
            }
            catch (...) {
                ROS_ERROR_STREAM("error");
            }
        }
    }
    else if (v[0] == "5") {
        if (v.size() == 1) {
            std::cout << "error" << std::endl;
        }
        else {
            try {
                final_change_trans.rotation.y = std::stod(v[1]);
            }
            catch (...) {
                ROS_ERROR_STREAM("error");
            }
            
        }
    }
    else if (v[0] == "6") {
        if (v.size() == 1) {
            std::cout << "error" << std::endl;
        }
        else {
            try {
                final_change_trans.rotation.z = std::stod(v[1]);
            }
            catch (...) {
                ROS_ERROR_STREAM("error");
            }
        }
    }
    else if (v[0] == "n") {
        final_change_trans.translation.x = -xyz_step;
    }
    else if (v[0] == "m") {
        final_change_trans.translation.x = xyz_step;
    }
    else if (v[0] == "j") {
        final_change_trans.translation.y = -xyz_step;
    }
    else if (v[0] == "k") {
        final_change_trans.translation.y = xyz_step;
    }
    else if (v[0] == "u") {
        final_change_trans.translation.z = -xyz_step;
    }
    else if (v[0] == "i") {
        final_change_trans.translation.z = xyz_step;
    }
    else if (v[0] == "c") {
        final_change_trans.rotation.x = -qxyz_step;
    }
    else if (v[0] == "v") {
        final_change_trans.rotation.x = qxyz_step;
    }
    else if (v[0] == "d") {
        final_change_trans.rotation.y = -qxyz_step;
    }
    else if (v[0] == "f") {
        final_change_trans.rotation.y = qxyz_step;
    }
    else if (v[0] == "e") {
        final_change_trans.rotation.z = -qxyz_step;
    }
    else if (v[0] == "r") {
        final_change_trans.rotation.z = qxyz_step;
    }
    else if (v[0] == "aq") {
        output.quit = true;
    }
    else {
        ROS_WARN_STREAM("parameter not collect");
    }
    output.transform = final_change_trans;
    return output;
}



/**
 * @brief ROSのTFを取得する
 *
 * @param target target_frame
 * @param source source_frame
 * @return geometry_msgs::Transform
 */
geometry_msgs::Transform TfBasic::tf_listen(std::string target, std::string source)
{
  geometry_msgs::TransformStamped final_tf;
  while (true)
  {
    try
    {
      final_tf = buffer_.lookupTransform(source, target, ros::Time(0));
      ROS_INFO_STREAM_ONCE("tf_listen: source: " << source << "  target: " << target);
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
 * @brief Transform型を簡単に作るための関数
 *
 * @param x
 * @param y
 * @param z
 * @param quaterion
 * @return geometry_msgs::Transform
 */
geometry_msgs::Transform TfBasic::make_geo_transform(double x, double y, double z, tf2::Quaternion quaterion)
{
  geometry_msgs::Transform output;
  output.translation.x = x;
  output.translation.y = y;
  output.translation.z = z;
  tf2::convert(quaterion, output.rotation);
  return output;
}

/*
1: child_frame
2: frame_id
3: transform
*/
geometry_msgs::TransformStamped TfBasic::make_geo_trans_stamped(std::string child_frame, std::string frame_id, geometry_msgs::Transform trans)
{
    geometry_msgs::TransformStamped outdata;
    outdata.child_frame_id = child_frame;
    outdata.header.frame_id = frame_id;
    outdata.transform = trans;
    return outdata;
}

/**
 * @brief xyzの指定した軸の方向に回転する
 *
 * @param rotated_quat クオータニオン型
 * @param xyz string型, "x" or "y" or "z"
 * @param angle
 * @return tf2::Quaternion
 */
tf2::Quaternion TfBasic::rotate_quaternion_by_axis(tf2::Quaternion rotated_quat, RotationOption option, double angle)
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

tf2::Quaternion TfBasic::rotate_xyz_make(double x, double y, double z, tf2::Quaternion q_moto)
{
  tf2::Quaternion quaternion;
  quaternion = rotate_quaternion_by_axis(q_moto, RotationOption::x, x) * q_moto;
  quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::y, y) * quaternion;
  quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::z, z) * quaternion;
  return quaternion;
}

void TfBasic::tf_data_show(geometry_msgs::Transform trans, std::string name)
{
    ROS_INFO_STREAM(name << "  x: " << trans.translation.x << "  y: " << trans.translation.y << " z: " << trans.translation.z);
    ROS_INFO_STREAM("qx: " << trans.rotation.x << " qy: " << trans.rotation.y << " qz: " << trans.rotation.z << " qw: " << trans.rotation.w);
}

tf2::Quaternion TfBasic::rotate_xyz_make(double x, double y, double z)
{
  tf2::Quaternion quaternion(0, 0, 0, 1);
  quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::x, x);
  quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::y, y);
  quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::z, z);
  return quaternion;
}

void TfBasic::broadcast(geometry_msgs::TransformStamped transform)
{
  
  for (int i = 0; i < 3; i++) {
    transform.header.stamp = ros::Time::now();
    br_.sendTransform(transform);
    ros::Duration(0.001).sleep();
  }
}

void TfBasic::static_broadcast(geometry_msgs::TransformStamped transform)
{
  
    for (int i = 0; i < 3; i++) {
        transform.header.stamp = ros::Time::now();
        static_br_.sendTransform(transform);
        ros::Duration(0.001).sleep();
    }
}

geometry_msgs::Quaternion TfBasic::make_geo_quaternion(tf2::Quaternion tf2_quat)
{
    geometry_msgs::Quaternion geo_quat;
    tf2::convert(tf2_quat, geo_quat);
    return geo_quat;
}

tf2::Quaternion TfBasic::make_tf2_quaternion(geometry_msgs::Quaternion geo_quat)
{
    tf2::Quaternion tf2_quat;
    tf2::convert(geo_quat, tf2_quat);
    return tf2_quat;
}