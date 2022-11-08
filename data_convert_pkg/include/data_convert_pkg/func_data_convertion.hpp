#pragma once
#include <util/util_base.hpp>
#include <util/util_sensor.hpp>
#include <common_msgs/BoxPosition.h>
#include <math.h>

struct ImageSize
{
    int width;
    int height;
};

struct Point2D
{
    double x;
    double y;
    Point2D();
    Point2D(double x1,double y1) {
        x = x1;
        y = y1;
    }
};

struct Point3D 
{
    double x;
    double y;
    double z;
    Point3D();
    Point3D(double x1, double y1, double z1) {
        x = x1;
        y = y1;
        z = z1;
    }
};

class FuncDataConvertion
{
public:
    FuncDataConvertion();
    Point2D func_3d_to_2d_pointcloud(Point3D, sensor_msgs::CameraInfo);
    Point2D func_3d_to_2d_coodinate(Point3D, sensor_msgs::CameraInfo);
    static bool point_is_in_image(Point2D, ImageSize, int);
    static ImageSize get_image_size(cv::Mat);
    XmlRpc::XmlRpcValue param_list;
private:
    ros::NodeHandle pnh_;
    double f_scale_, cx_scale_, cy_scale_;
};