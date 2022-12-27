#include <visualize_package/visualize_client.hpp>

VisualizeClient::VisualizeClient(ros::NodeHandle &nh):
nh_(nh),
pnh_("~")
{
    set_parameter();
    hdf5_client_ = nh_.serviceClient<common_srvs::Hdf5OpenAccService>(hdf5_open_acc_service_name_);
    visualize_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(visualize_service_name_);
    vis_img_client_ = nh_.serviceClient<common_srvs::VisualizeImage>(vis_img_service_name_);
}

void VisualizeClient::set_parameter()
{
    pnh_.getParam("visualize_client", param_list);
    hdf5_open_acc_service_name_ = static_cast<std::string>(param_list["hdf5_open_acc_service_name"]);
    Util::message_show("hdf5_open_acc_service", hdf5_open_acc_service_name_);
    visualize_service_name_ = static_cast<std::string>(param_list["visualize_service_name"]);
    vis_img_service_name_ = static_cast<std::string>(param_list["visualize_image_service_name"]);
    hdf5_open_file_path_ = static_cast<std::string>(param_list["hdf5_open_file_path"]);
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    cloud_process_.set_crop_frame(sensor_frame_, world_frame_);
    pnh_.getParam("crop", crop_);
}

void VisualizeClient::main()
{
    int index = 0;
    while (1) {
        common_srvs::Hdf5OpenAccService hdf5_srv;
        hdf5_srv.request.index = index;
        hdf5_srv.request.hdf5_open_file_path = hdf5_open_file_path_;
        hdf5_srv.request.is_reload = true;
        Util::message_show("index", index);
        Util::client_request(hdf5_client_, hdf5_srv, hdf5_open_acc_service_name_);
        common_srvs::VisualizeCloud visualize_srv;
        common_srvs::VisualizeImage visualize_img_srv;
        common_msgs::CloudData cloud_data = hdf5_srv.response.cloud_data;
        if (crop_) {
            pcl::PointCloud<pcl::PointXYZL> pcl_label_data = UtilMsgData::cloudmsg_to_pclLabel(cloud_data);
            pcl_label_data = cloud_process_.cropbox_segmenter(pcl_label_data);
            cloud_data = UtilMsgData::pclLabel_to_cloudmsg(pcl_label_data);
        }
        visualize_srv.request.cloud_data_list.push_back(cloud_data);
        visualize_srv.request.topic_name_list.push_back("index_" + std::to_string(index));
        Util::client_request(visualize_client_, visualize_srv, visualize_service_name_);
        visualize_img_srv.request.image_list.push_back(hdf5_srv.response.image);
        visualize_img_srv.request.topic_name_list.push_back("image_topic_" + std::to_string(index));
        Util::client_request(vis_img_client_, visualize_img_srv, vis_img_service_name_);
        if (index >= hdf5_srv.response.data_size - 1) {
            break;
        }
        index++;
    }
}

