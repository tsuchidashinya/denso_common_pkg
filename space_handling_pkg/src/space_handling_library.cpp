#include <space_handling_pkg/space_handling_library.hpp>



SpaceHandlingLibrary::SpaceHandlingLibrary() : pnh_("~")
{
    set_parameter();
}

void SpaceHandlingLibrary::set_parameter()
{
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
}

common_msgs::CloudData SpaceHandlingLibrary::search_nearest_point(common_msgs::CloudData sensor_cloud, common_msgs::CloudData mesh_cloud, int instance, double radious = 0.004)
{
    pcl::PointCloud<PclXyz> sensor_pcl, mesh_pcl;
    sensor_pcl = UtilMsgData::cloudmsg_to_pcl(sensor_cloud);
    mesh_pcl = UtilMsgData::cloudmsg_to_pcl(mesh_cloud);
    pcl::search::KdTree<PclXyz> kdtree;
    kdtree.setInputCloud(sensor_pcl.makeShared());
    std::vector<int> pointIndices;
    std::vector<float> squaredDistance;
    ros::WallTime start = ros::WallTime::now();
    for (auto mesh : mesh_pcl.points) {
        if (kdtree.radiusSearch(mesh, radious, pointIndices, squaredDistance)) {
            for (int j = 0; j < pointIndices.size(); j++) {
                ros::WallTime end = ros::WallTime::now();
                ros::WallDuration calc_time = end - start;
                if (calc_time.toSec() >= 2) {
                    ROS_WARN_STREAM("nearest point search failed");
                    return sensor_cloud;
                }
                else {
                    sensor_cloud.instance[pointIndices[j]] = instance;
                }
            }
        }
        pointIndices.clear();
        squaredDistance.clear();
    }
    return sensor_cloud;
}

common_msgs::CloudData SpaceHandlingLibrary::search_nearest_point_on_unit(common_msgs::CloudData sensor_cloud, common_msgs::CloudData search_cloud, int index, double radious = 0.004)
{
    pcl::PointCloud<PclXyz> sensor_pcl;
    sensor_pcl = UtilMsgData::cloudmsg_to_pcl(sensor_cloud);
    PclXyz search_point;
    search_point.x = search_cloud.x[index];
    search_point.y = search_cloud.y[index];
    search_point.z = search_cloud.z[index];
    pcl::search::KdTree<PclXyz> kdtree;
    kdtree.setInputCloud(sensor_pcl.makeShared());
    std::vector<int> pointIndices;
    std::vector<float> squaredDistance;
    ros::WallTime start = ros::WallTime::now();
    common_msgs::CloudData out_cloud;
    if (kdtree.radiusSearch(search_point, radious, pointIndices, squaredDistance)) {
        for (int j = 0; j < pointIndices.size(); j++) {
            ros::WallTime end = ros::WallTime::now();
            ros::WallDuration calc_time = end - start;
            if (calc_time.toSec() >= 2) {
                ROS_WARN_STREAM("nearest point search failed");
                return out_cloud;
            }
            else {
                out_cloud.x.push_back(sensor_cloud.x[pointIndices[j]]);
                out_cloud.y.push_back(sensor_cloud.y[pointIndices[j]]);
                out_cloud.z.push_back(sensor_cloud.z[pointIndices[j]]);
                out_cloud.instance.push_back(sensor_cloud.instance[pointIndices[j]]);
            }
        }
    }
    return out_cloud;
}


std::vector<common_msgs::ObjectInfo> SpaceHandlingLibrary::extract_occuluder(std::vector<common_msgs::ObjectInfo> object_info)
{
    std::vector<common_msgs::ObjectInfo> out_data;
    out_data = detect_occuluder(object_info);
    return out_data;
}

std::vector<common_msgs::ObjectInfo> SpaceHandlingLibrary::detect_occuluder(std::vector<common_msgs::ObjectInfo> object_info_list)
{
    std::vector<common_msgs::ObjectInfo> occluder_list;
    int collision = 0;
    for (int i = 0; i < object_info_list.size(); i++) {
       geometry_msgs::Transform trans_get;
       object_info_list[i].position = tf_func_.tf_listen(object_info_list[i].tf_name, world_frame_);
       if (i == 0) {
        occluder_list.push_back(object_info_list[i]);
       }
       else {
        collision = 0;
        std::vector<int> delete_list;
        for (int k = 0; k < occluder_list.size(); k++) {
            double dis = Util::distance(object_info_list[i].position.translation.x, object_info_list[i].position.translation.y, occluder_list[k].position.translation.x, occluder_list[k].position.translation.y);
            if (dis < object_info_list[i].occulusion_radious + occluder_list[k].occulusion_radious) {
                if (object_info_list[i].position.translation.z > occluder_list[k].position.translation.z) {
                    delete_list.push_back(occluder_list[k].index);
                }
                else {
                    collision = 1;
                    break;
                }
            }
        }
        for (int i = 0; i < delete_list.size(); i++) {
            for (int j = 0; j < occluder_list.size(); j++) {
                if (occluder_list[j].index == delete_list[i]) {
                    occluder_list.erase(occluder_list.begin() + j);
                    break;
                }
            }
        }
        if (collision == 0) {
            occluder_list.push_back(object_info_list[i]);
        }
       } 
    }
    return occluder_list;
}

