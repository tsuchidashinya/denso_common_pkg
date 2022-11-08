#pragma once
#include <util/common_header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

struct KeyBoardTf
{
    geometry_msgs::Transform transform;
    bool quit;
};

enum RotationOption
{
    x,
    y,
    z
};

class TfBasic
{
public:
    TfBasic();
    geometry_msgs::Transform get_tf(std::string, std::string);
    static tf2::Quaternion rotate_quaternion_by_axis(tf2::Quaternion, RotationOption, double);
    static tf2::Quaternion rotate_xyz_make(double, double, double, tf2::Quaternion);
    static tf2::Quaternion rotate_xyz_make(double, double, double);
    static geometry_msgs::Quaternion make_geo_quaternion(tf2::Quaternion);
    static tf2::Quaternion make_tf2_quaternion(geometry_msgs::Quaternion);
    void static_broadcast(geometry_msgs::TransformStamped);
    void broadcast(geometry_msgs::TransformStamped);
    KeyBoardTf get_keyboard_tf(double, double);
    static geometry_msgs::Transform make_geo_transform(double, double, double, tf2::Quaternion);
    static geometry_msgs::TransformStamped make_geo_trans_stamped(std::string, std::string, geometry_msgs::Transform);
private:
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::StaticTransformBroadcaster static_br_;
    tf2_ros::TransformBroadcaster br_;
};