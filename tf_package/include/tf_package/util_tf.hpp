#pragma once
#include "common_header.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

enum RotationOption
{
    x,
    y,
    z
};

class UtilTF
{
public:
    UtilTF();
    geometry_msgs::Transform get_tf(std::string, std::string);
    static tf2::Quaternion rotate_quaternion_by_axis(tf2::Quaternion, RotationOption, double);
    static tf2::Quaternion rotate_xyz_make(double, double, double, tf2::Quaternion);
    static tf2::Quaternion rotate_xyz_make(double, double, double);
    void static_broadcast(geometry_msgs::TransformStamped);
    void broadcast(geometry_msgs::TransformStamped);

private:
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::StaticTransformBroadcaster static_br_;
    tf2_ros::TransformBroadcaster br_;
};