#pragma once
#include <util_package/common_header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

struct KeyBoardTf
{
    float x_add = 0;
    float y_add = 0;
    float z_add = 0;
    float qx_add = 0;
    float qy_add = 0;
    float qz_add = 0;
    bool quit = false;
};

enum RotationOption
{
    x,
    y,
    z
};

class TfFunction
{
public:
    TfFunction();
    static geometry_msgs::Transform change_tf_frame_by_rotate(geometry_msgs::Transform, geometry_msgs::Transform);
    geometry_msgs::Transform tf_listen(std::string, std::string);
    static tf2::Quaternion rotate_quaternion_by_axis(tf2::Quaternion, RotationOption, double);
    static tf2::Quaternion rotate_xyz_make(double, double, double, tf2::Quaternion);
    static tf2::Quaternion rotate_xyz_make(double, double, double);
    static geometry_msgs::Quaternion tf2_quat_to_geo_quat(tf2::Quaternion);
    static tf2::Quaternion geo_quat_to_tf2_quat(geometry_msgs::Quaternion);
    void static_broadcast(geometry_msgs::TransformStamped);
    void broadcast(geometry_msgs::TransformStamped);
    geometry_msgs::Transform add_keyboard_tf(geometry_msgs::Transform, KeyBoardTf);
    KeyBoardTf get_keyboard_tf(double, double);
    static geometry_msgs::Transform make_geo_transform(double, double, double, tf2::Quaternion);
    static geometry_msgs::TransformStamped make_geo_trans_stamped(std::string, std::string, geometry_msgs::Transform);
    static void tf_data_show(geometry_msgs::Transform, std::string);
private:
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::StaticTransformBroadcaster static_br_;
    tf2_ros::TransformBroadcaster br_;
};