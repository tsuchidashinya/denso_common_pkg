/**
 * @file decide_object_position.cpp
 * @author tsuchidashinya (tsuchida.shinya413@mail.kyutech.jp)
 * @brief
 * @version 0.1
 * @date 2022-09-11
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <gazebo_model_pkg/decide_object_position.hpp>

DecidePosition::DecidePosition() : 
pnh_("~"),
decide_pose_option_(DecidePoseOption::Head),
sensor_small_deviation_(0)
{
    set_parameter();
}

common_srvs::SetLinkVisualPropertiesRequest DecidePosition::get_link_visual_parameter(std::string link_name)
{
    common_srvs::SetLinkVisualPropertiesRequest request;
    request.link_name = link_name;
    auto color_min = 0.1, color_max = 0.9;
    auto red = Util::random_float_static(color_min, color_max);
    auto blue = Util::random_float_static(color_min, color_max);
    auto green = Util::random_float_static(color_min, color_max);
    // auto a = Util::random_float_static(0, 1);
    auto a = 1;
    request.ambient.r = red;
    request.ambient.b = blue;
    request.ambient.g = green;
    request.ambient.a = a;
    request.diffuse.r = red;
    request.diffuse.b = blue;
    request.diffuse.g = green;
    request.diffuse.a = a;
    request.emissive.r = 0;
    request.emissive.g = 0;
    request.emissive.b = 0;
    request.emissive.a = 1;
    return request;
}

void DecidePosition::set_decice_pose_option(DecidePoseOption option)
{
    if (option == DecidePoseOption::FullRandom) {
        decide_pose_option_ = DecidePoseOption::FullRandom;
    }
    else if (option == DecidePoseOption::Head) {
        decide_pose_option_ = DecidePoseOption::Head;
    }
    else if (option == DecidePoseOption::HeadAndTail) {
        decide_pose_option_ = DecidePoseOption::HeadAndTail;
    }
}

void DecidePosition::set_parameter()
{
    pnh_.getParam("decide_object_position", param_list);
    z_position_ = param_list["z_position"];
    box_height_ = param_list["box_height"];
    box_name_ = static_cast<std::string>(param_list["box_name"]);
    sensor_name_ = static_cast<std::string>(param_list["sensor_name"]);
    sensor_angle_max_ = param_list["sensor_angle_max"];
    sensor_angle_min_ = param_list["sensor_angle_min"];
    sensor_distance_min_ = param_list["sensor_distance_min"];
    sensor_distance_max_ = param_list["sensor_distance_max"];
    object_height_ = param_list["object_height"];
    sensor_small_deviation_ = param_list["sensor_small_deviation"];
    // std::string para_decide = static_cast<std::string>("decide_pose_opition");
    // if (para_decide == "FullRandom") {
    //     decide_pose_option_ = DecidePoseOption::FullRandom;
    // }
    // else if (para_decide == "HeadAndTail") {
    //     decide_pose_option_ = DecidePoseOption::HeadAndTail;
    // }
}


common_msgs::ObjectInfo DecidePosition::make_object_info(int object_id, std::string object_name)
{
    common_msgs::ObjectInfo outdata;
    outdata.tf_name = object_name + "_" + std::to_string(object_id);
    outdata.object_name = object_name;
    return outdata;
}

gazebo_msgs::SetLightPropertiesRequest DecidePosition::get_light_properties()
{
    gazebo_msgs::SetLightPropertiesRequest request;
    request.light_name = "sun";
    request.cast_shadows = true;
    // request.attenuation_constant = 0.9;
    // request.attenuation_linear = 0.01;
    // request.attenuation_quadratic = 0;
    request.attenuation_constant = Util::random_float_static(0, 1);
    request.attenuation_linear = Util::random_float_static(0, 1);
    request.attenuation_quadratic = Util::random_float_static(0, 1);
    request.direction.x = Util::random_float_static(-0.99, 0.99);
    request.direction.y = Util::random_float_static(-0.99, 0.99);
    request.direction.z = -0.99;
    request.pose.orientation.x = 0;
    request.pose.orientation.y = 0;
    request.pose.orientation.z = 0;
    request.pose.orientation.w = 1;
    request.pose.position.x = Util::random_float_static(-10, 10);
    request.pose.position.y = Util::random_float_static(-10, 10);
    request.pose.position.z = 10;
    float color_max, color_min;
    color_min = 0.01;
    color_max = 0.99;
    request.diffuse.r = Util::random_float_static(color_min, color_max);
    request.diffuse.g = Util::random_float_static(color_min, color_max);
    request.diffuse.b = Util::random_float_static(color_min, color_max);
    request.diffuse.a = Util::random_float_static(0, 1);
    request.specular.r = Util::random_float_static(color_min, color_max);
    request.specular.g = Util::random_float_static(color_min, color_max);
    request.specular.b = Util::random_float_static(color_min, color_max);
    request.specular.a = Util::random_float_static(0, 1);
    return request;
}

/*
1: index
2: object_id
3: object_option(ObjectListType)
*/
common_msgs::ObjectInfo DecidePosition::make_object_info(int index, int object_id, ObjectListType object_option)
{
    common_msgs::ObjectInfo object;
    object.index = index;
    object.object_name = object_option.object_name;
    object.radious = object_option.radious;
    object.occulusion_radious = object_option.occlusion_radious;
    object.tf_name = object_option.object_name + "_" + std::to_string(object_id);
    object.instance_num = object_option.instance_num;
    return object;
}

common_msgs::ObjectInfo DecidePosition::make_object_info(std::string object_name, std::string tf_name)
{
    common_msgs::ObjectInfo outdata;
    outdata.object_name = object_name;
    outdata.tf_name = tf_name;
    return outdata;
}

/**
 * @brief オブジェクトの配置を決定する
 *
 * @return std::vector<common_msgs::ObjectInfo>
 */
std::vector<common_msgs::ObjectInfo> DecidePosition::get_randam_place_position(std::vector<common_msgs::ObjectInfo> object_info)
{
    {
        int count = 0;
        int map_index = 0;
        std::map<int, int> check_point;
        check_point[map_index] = 0;
        bool loop_ok;
        double x, y, x_range = 0.08, y_range = 0.12;
        Util util;
        for (int i = 0; i < object_info.size(); i++)
        {
            count = 0;
            if (i > 0)
            {
                if (i % 4 == 0)
                {
                    map_index++;
                    check_point[map_index] = i;
                }
                do
                {
                    loop_ok = true;

                    count++;
                    if (count == 2000)
                    {
                        ROS_WARN_STREAM("Over");
                        break;
                    }
                    // ros_print_parameter("loop_ok");
                    x = util.random_float(-x_range, x_range);
                    y = util.random_float(-y_range, y_range);

                    for (int j = check_point[map_index]; j < i; j++)
                    {
                        if (Util::distance(x, y, object_info[j].position.translation.x, object_info[j].position.translation.y) < object_info[i].radious + object_info[j].radious)
                            loop_ok = false;
                    }

                } while (!loop_ok);
            }
            else
            {
                x = util.random_float(-x_range, x_range);
                y = util.random_float(-y_range, y_range);
            }

            double z = z_position_ + box_height_ + map_index * 0.05;
            double roll, pitch, yaw;
            float scale_para = 20;
            if (decide_pose_option_ == DecidePoseOption::FullRandom) {
                roll = util.random_float(-M_PI, M_PI);
                pitch = util.random_float(-M_PI, M_PI);
            }
            else if (decide_pose_option_ == DecidePoseOption::Head) {
                roll = util.random_float(-M_PI / scale_para, M_PI / scale_para);
                pitch = util.random_float(-M_PI / scale_para, M_PI / scale_para);
            }
            else if (decide_pose_option_ == DecidePoseOption::HeadAndTail) {
                if (util.probability() < 0.7) {
                    roll = util.random_float(-M_PI / scale_para, M_PI / scale_para);
                    pitch = util.random_float(-M_PI / scale_para, M_PI / scale_para);
                }
                else {
                    roll = util.random_float(-M_PI / scale_para + M_PI, M_PI / scale_para + M_PI);
                    pitch = util.random_float(-M_PI / scale_para, M_PI / scale_para);
                }
            }
            // ROS_INFO_STREAM("x: " << x << "  y: " << y << "z: " << z);
            object_info[i].position = TfFunction::make_geo_transform(x, y, z, TfFunction::rotate_xyz_make(roll, pitch, yaw));
            
        }
    }
    return object_info;
}

std::vector<common_msgs::ObjectInfo> DecidePosition::get_remove_position(std::vector<common_msgs::ObjectInfo> object_info)
{
    Util util;
    for (int i = 0; i < object_info.size(); i++)
    {
        double x = util.random_float(10, 20);
        double y = util.random_float(10, 20);
        double z = util.random_float(0, 20);
        tf2::Quaternion quaternion = TfFunction::rotate_xyz_make(0, 0, 0);
        object_info[i].position =  TfFunction::make_geo_transform(x, y, z, quaternion);
    }
    return object_info;
}

common_msgs::ObjectInfo DecidePosition::get_box_position()
{
    Util util;
    common_msgs::ObjectInfo outdata;
    double x = 0;
    double y = 0;
    double z = z_position_;
    tf2::Quaternion quat = TfFunction::rotate_xyz_make(0, 0, 0);
    outdata.position = TfFunction::make_geo_transform(x, y, z, quat);
    outdata.object_name = box_name_;
    outdata.tf_name = box_name_;
    return outdata;
}

common_msgs::ObjectInfo DecidePosition::get_sensor_position()
{
    Util util;
    common_msgs::ObjectInfo outdata;
    double angle = util.random_float(sensor_angle_min_, sensor_angle_max_);
    double sensor_distance = util.random_float(sensor_distance_min_, sensor_distance_max_);
    double x, y, z;
    tf2::Quaternion quaternion;
    auto sensor_scale = sensor_distance / sensor_distance_max_;
    sensor_scale = sensor_scale * sensor_scale;
    auto sensor_small_deviation_result = sensor_small_deviation_ * sensor_scale;
    auto small_deviation_x = util.random_float(-sensor_small_deviation_result, sensor_small_deviation_result);
    auto small_deviation_y = util.random_float(-sensor_small_deviation_result, sensor_small_deviation_result);
    // auto small_deviation_x = sensor_small_deviation_ * sensor_scale;
    // auto small_deviation_y = sensor_small_deviation_ * sensor_scale;
    // Util::message_show("small_deviation_x", small_deviation_x);
    // Util::message_show("small_deviation_y", small_deviation_y);
    x = sensor_distance * sin(angle) + small_deviation_x;
    y = small_deviation_y;
    z = sensor_distance * cos(angle);
    quaternion = TfFunction::rotate_xyz_make(0, angle, 0);
    
    outdata.position = TfFunction::make_geo_transform(x, y, z, quaternion);
    outdata.object_name = sensor_name_;
    outdata.tf_name = sensor_name_;
    return outdata;
}


common_msgs::ObjectInfo DecidePosition::get_sensor_return_position()
{
    Util util;
    common_msgs::ObjectInfo outdata;
    double angle = (sensor_angle_max_ + sensor_angle_min_) / 2;
    double sensor_distance = (sensor_distance_min_ + sensor_distance_max_) / 2;
    double x, y, z;
    tf2::Quaternion quaternion;
    auto sensor_scale = sensor_distance / sensor_distance_max_;
    sensor_scale = sensor_scale * sensor_scale;
    auto sensor_small_deviation_result = sensor_small_deviation_ * sensor_scale;
    auto small_deviation_x = util.random_float(-sensor_small_deviation_result, sensor_small_deviation_result);
    auto small_deviation_y = util.random_float(-sensor_small_deviation_result, sensor_small_deviation_result);
    // auto small_deviation_x = sensor_small_deviation_ * sensor_scale;
    // auto small_deviation_y = sensor_small_deviation_ * sensor_scale;
    // Util::message_show("small_deviation_x", small_deviation_x);
    // Util::message_show("small_deviation_y", small_deviation_y);
    x = sensor_distance * sin(angle);
    y = 0;
    z = sensor_distance * cos(angle);
    quaternion = TfFunction::rotate_xyz_make(0, angle, 0);
    
    outdata.position = TfFunction::make_geo_transform(x, y, z, quaternion);
    outdata.object_name = sensor_name_;
    outdata.tf_name = sensor_name_;
    return outdata;
}
