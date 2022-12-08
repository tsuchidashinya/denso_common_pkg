#include <tf_package/tf_function.hpp>

TfFunction::TfFunction() :
    buffer_(),
    listener_(buffer_)
{

}

/*
1: double: xyz_step
2: double: qxyz_step
*/
KeyBoardTf TfFunction::get_keyboard_tf(double xyz_step, double qxyz_step)
{
    KeyBoardTf output;
    output.quit = false;
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
                output.x_add = std::stod(v[1]);
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
                output.y_add = std::stod(v[1]);
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
                output.z_add = std::stod(v[1]);
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
                output.qx_add = std::stod(v[1]);
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
                output.qy_add = std::stod(v[1]);
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
                output.qz_add = std::stod(v[1]);
            }
            catch (...) {
                ROS_ERROR_STREAM("error");
            }
        }
    }
    else if (v[0] == "n") {
        output.x_add = -xyz_step;
    }
    else if (v[0] == "m") {
        output.x_add = xyz_step;
    }
    else if (v[0] == "j") {
        output.y_add = -xyz_step;
    }
    else if (v[0] == "k") {
        output.y_add = xyz_step;
    }
    else if (v[0] == "u") {
        output.z_add = -xyz_step;
    }
    else if (v[0] == "i") {
        output.z_add = xyz_step;
    }
    else if (v[0] == "c") {
        output.qx_add = -qxyz_step;
    }
    else if (v[0] == "v") {
        output.qx_add = qxyz_step;
    }
    else if (v[0] == "d") {
        output.qy_add = -qxyz_step;
    }
    else if (v[0] == "f") {
        output.qy_add = qxyz_step;
    }
    else if (v[0] == "e") {
        output.qz_add = -qxyz_step;
    }
    else if (v[0] == "r") {
        output.qz_add = qxyz_step;
    }
    else if (v[0] == "aq") {
        output.quit = true;
    }
    else {
        ROS_WARN_STREAM("parameter not collect");
    }
    return output;
}

geometry_msgs::Transform TfFunction::add_keyboard_tf(geometry_msgs::Transform previous_trans, KeyBoardTf add_key)
{   
    geometry_msgs::Transform out_tf;
    out_tf.translation.x = previous_trans.translation.x + add_key.x_add;
    out_tf.translation.y = previous_trans.translation.y + add_key.y_add;
    out_tf.translation.z = previous_trans.translation.z + add_key.z_add;
    tf2::Quaternion quaternion;
    tf2::convert(previous_trans.rotation, quaternion);
    if (add_key.qx_add != 0) {
        quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::x, add_key.qx_add) * quaternion;
    }
    else if (add_key.qy_add != 0) {
        quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::y, add_key.qy_add) * quaternion;
    }
    else if (add_key.qz_add != 0) {
        quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::z, add_key.qz_add) * quaternion;
    }
    tf2::convert(quaternion, out_tf.rotation);
    return out_tf;
}



/**
 * @brief ROSのTFを取得する
 *
 * @param target target_frame
 * @param source source_frame
 * @return geometry_msgs::Transform
 */
geometry_msgs::Transform TfFunction::tf_listen(std::string target, std::string source)
{
  geometry_msgs::TransformStamped final_tf;
  while (true)
  {
    try
    {
      final_tf = buffer_.lookupTransform(source, target, ros::Time(0));
      break;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("target: " << target << "   source: " << source);
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
geometry_msgs::Transform TfFunction::make_geo_transform(double x, double y, double z, tf2::Quaternion quaterion)
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
geometry_msgs::TransformStamped TfFunction::make_geo_trans_stamped(std::string child_frame, std::string frame_id, geometry_msgs::Transform trans)
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
tf2::Quaternion TfFunction::rotate_quaternion_by_axis(tf2::Quaternion rotated_quat, RotationOption option, double angle)
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

geometry_msgs::Transform TfFunction::change_tf_frame_by_rotate(geometry_msgs::Transform tf_ori, geometry_msgs::Transform tf_add)
{
    geometry_msgs::Transform out_tf;
    // out_tf.rotation = tf_ori.rotation;
    tf2::Quaternion q_ori(tf_ori.translation.x, tf_ori.translation.y, tf_ori.translation.z, 0), q_after, q_rotate, q_rotate_ori;
    tf2::convert(tf_ori.rotation, q_rotate_ori);
    tf2::convert(tf_add.rotation, q_rotate);
    q_rotate_ori = q_rotate * q_rotate_ori;
    tf2::convert(q_rotate_ori, out_tf.rotation);
    q_after = q_rotate.inverse() * q_ori * q_rotate;
    out_tf.translation.x = q_after[0] + tf_add.translation.x;
    out_tf.translation.y = q_after[1] + tf_add.translation.y;
    out_tf.translation.z = q_after[2] + tf_add.translation.z;
    return out_tf;
}

tf2::Quaternion TfFunction::rotate_xyz_make(double x, double y, double z, tf2::Quaternion q_moto)
{
  tf2::Quaternion quaternion;
  quaternion = rotate_quaternion_by_axis(q_moto, RotationOption::x, x) * q_moto;
  quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::y, y) * quaternion;
  quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::z, z) * quaternion;
  return quaternion;
}

void TfFunction::tf_data_show(geometry_msgs::Transform trans, std::string name)
{
    ROS_INFO_STREAM(name << "  x: " << trans.translation.x << "  y: " << trans.translation.y << " z: " << trans.translation.z);
    ROS_INFO_STREAM("qx: " << trans.rotation.x << " qy: " << trans.rotation.y << " qz: " << trans.rotation.z << " qw: " << trans.rotation.w);
}

tf2::Quaternion TfFunction::rotate_xyz_make(double x, double y, double z)
{
  tf2::Quaternion quaternion(0, 0, 0, 1);
  quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::x, x) * quaternion;
  quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::y, y) * quaternion;
  quaternion = rotate_quaternion_by_axis(quaternion, RotationOption::z, z) * quaternion;
  return quaternion;
}

void TfFunction::broadcast(geometry_msgs::TransformStamped transform)
{
  
//   for (int i = 0; i < 2; i++) {
    transform.header.stamp = ros::Time::now();
    br_.sendTransform(transform);
    // ros::Duration(0.001).sleep();
//   }
}

void TfFunction::static_broadcast(geometry_msgs::TransformStamped transform)
{
  
    // for (int i = 0; i < 2; i++) {
        transform.header.stamp = ros::Time::now();
        static_br_.sendTransform(transform);
        // ros::Duration(0.001).sleep();
    // }
}

geometry_msgs::Quaternion TfFunction::make_geo_quaternion(tf2::Quaternion tf2_quat)
{
    geometry_msgs::Quaternion geo_quat;
    tf2::convert(tf2_quat, geo_quat);
    return geo_quat;
}

tf2::Quaternion TfFunction::make_tf2_quaternion(geometry_msgs::Quaternion geo_quat)
{
    tf2::Quaternion tf2_quat;
    tf2::convert(geo_quat, tf2_quat);
    return tf2_quat;
}