#include <data_transform_pkg/make_2Dinfo_by_3D.hpp>

Make2DInfoBy3D::Make2DInfoBy3D(sensor_msgs::CameraInfo cinfo, ImageSize img_size): pnh_("~")
{
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("make_2Dinfo_by_3D", param_list);
    object_radious_x_ = param_list["radious_x"];
    object_radious_y_ = param_list["radious_y"];
    cinfo_ = cinfo;
    img_size_ = img_size;
}

std::vector<common_msgs::BoxPosition> Make2DInfoBy3D::get_out_data(std::vector<std::string> tf_frames)
{
    std::vector<Point3D> point_3d_datas = get_3Dpoint_from_sensor(tf_frames);
    return convert_3Dto2D(point_3d_datas, tf_frames);
}

std::vector<common_msgs::BoxPosition> Make2DInfoBy3D::convert_3Dto2D(std::vector<Point3D> point3D_objects, std::vector<std::string> tf_names)
{
    std::vector<common_msgs::BoxPosition> outdata;
    for (int i = 0; i < point3D_objects.size(); i++) {
        common_msgs::BoxPosition b_box;
        double x = point3D_objects[i].x;
        double y = point3D_objects[i].y;
        double z = point3D_objects[i].z;
        Point3D pt_cv(y, x, z);
        Point3D pt_cv_left(y - object_radious_y_, x - object_radious_x_, z);
        Point3D pt_cv_right(y + object_radious_y_, x + object_radious_x_, z);
        Point2D uv_left, uv_right;
        FuncDataConvertion data_convert;
        uv_left = data_convert.func_3d_to_2d_coodinate(pt_cv_left, cinfo_);
        uv_right = data_convert.func_3d_to_2d_coodinate(pt_cv_right, cinfo_);
        if (FuncDataConvertion::point_is_in_image(uv_left, img_size_, 1) && FuncDataConvertion::point_is_in_image(uv_right, img_size_, 1)) {
            b_box.x_one = uv_left.x;
            b_box.x_two = uv_right.x;
            b_box.y_one = uv_left.y;
            b_box.y_two = uv_right.y;
            b_box.tf_name = tf_names[i];
            outdata.push_back(b_box);
            // Util::message_show("tf_id", i);
        }
        else {
           std::cout << "Not Detect" << ": " << uv_left.x << " " << uv_left.y << " " << uv_right.x << " " << uv_right.y << std::endl;
        }
    }
    return outdata;
}

cv::Mat Make2DInfoBy3D::draw_b_box(cv::Mat img, std::vector<common_msgs::BoxPosition> b_boxs)
{
    for (int i = 0; i < b_boxs.size(); i++) {
        double uv_lect_x = b_boxs[i].x_one;
        double uv_left_y = b_boxs[i].y_one;
        double uv_right_x = b_boxs[i].x_two;
        double uv_right_y = b_boxs[i].y_two;
        // std::cout << i << ": " << uv_lect_x << " " << uv_left_y << " " << uv_right_x << " " << uv_right_y << std::endl;
        cv::rectangle(img, cv::Point(uv_lect_x, uv_right_y), cv::Point(uv_right_x, uv_left_y), cv::Scalar(0, 255, 0), 3);
        cv::circle(img, cv::Point(uv_lect_x, uv_right_y), 8, cv::Scalar(255, 255, 255), 3, 1);
        cv::circle(img, cv::Point(uv_right_x, uv_left_y), 8, cv::Scalar(0, 0, 0), 3, 1);
    }
    return img;
}

std::vector<Point3D> Make2DInfoBy3D::get_3Dpoint_from_sensor(std::vector<std::string> tf_frames)
{
    std::vector<Point3D> outdata;
    outdata.resize(tf_frames.size());
    geometry_msgs::Transform object_tf, sensor_tf; 
    sensor_tf = tf_basic_.tf_listen(sensor_frame_, world_frame_);
    for (int i = 0; i < tf_frames.size(); i++) {
        sensor_tf = tf_basic_.tf_listen(sensor_frame_, world_frame_);
        // TfBasic::tf_data_show(sensor_tf, sensor_frame_);
        // Util::message_show(world_frame_, sensor_frame_);
        tf2::Quaternion source_quat;
        tf2::convert(sensor_tf.rotation, source_quat);
        object_tf = tf_basic_.tf_listen(tf_frames[i], world_frame_);
        // TfBasic::tf_data_show(object_tf, tf_frames[i]);
        double x = object_tf.translation.x - sensor_tf.translation.x;
        double y = object_tf.translation.y - sensor_tf.translation.y;
        double z = object_tf.translation.z - sensor_tf.translation.z;
        tf2::Quaternion q_before(x, y, z, 0), q_after, q_convert;
        q_convert = TfBasic::rotate_quaternion_by_axis(source_quat, RotationOption::z, -M_PI/2) * source_quat;
        q_after = q_convert * q_before * q_convert.inverse();
        outdata[i].x = q_after.x();
        outdata[i].y = q_after.y();
        outdata[i].z = q_after.z();
        // std::cout << outdata[i].x << "  " << outdata[i].y << "  " << outdata[i].y << std::endl;
    }
    return outdata;
}
