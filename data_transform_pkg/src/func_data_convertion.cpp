#include <data_transform_pkg/func_data_convertion.hpp>

FuncDataConvertion::FuncDataConvertion() : 
pnh_("~")
{
    pnh_.getParam("data_transform_pkg", param_list);
    f_scale_ = param_list["f_scale"];
    cx_scale_ = param_list["cx_scale"];
    cy_scale_ = param_list["cy_scale"];
}

bool FuncDataConvertion::point_is_in_image(Point2D point2d, ImageSize im_size, int scale)
{
    return point2d.x >= 0 && point2d.x < (im_size.width / scale) && point2d.y >= 0 && point2d.y < (im_size.height / scale);
}

ImageSize FuncDataConvertion::get_image_size(cv::Mat img)
{
    ImageSize outdata;
    outdata.height = img.rows;
    outdata.width = img.cols;
    return outdata;
}

Point2D FuncDataConvertion::func_3d_to_2d_pointcloud(Point3D xyz, sensor_msgs::CameraInfo cinfo_msg)
{
    double fx, tx, cx, fy, ty, cy;
    fx = cinfo_msg.K[0] * f_scale_;
    tx = cinfo_msg.K[1];
    cx = cinfo_msg.K[2] * cx_scale_;
    fy = cinfo_msg.K[4] * f_scale_;
    ty = cinfo_msg.K[3];
    cy = cinfo_msg.K[5] * cy_scale_;
    Point2D output;
    output.x = (fx * xyz.x + tx) / xyz.z + cx;
    output.y = (fy * xyz.y + ty) / xyz.z + cy;
    return output;
}

Point2D FuncDataConvertion::func_3d_to_2d_coodinate(Point3D xyz, sensor_msgs::CameraInfo cinfo_msg)
{
    double fx, tx, cx, fy, ty, cy;
    fx = cinfo_msg.K[0] * f_scale_;
    tx = cinfo_msg.K[1];
    cx = cinfo_msg.K[2] * cx_scale_;
    fy = cinfo_msg.K[4] * f_scale_;
    ty = cinfo_msg.K[3];
    cy = cinfo_msg.K[5] * cy_scale_;
    Point2D output;
    output.x = (fx * xyz.x + tx) / xyz.z + cx;
    output.y = (fy * -xyz.y + ty) / xyz.z + cy;
    return output;
}