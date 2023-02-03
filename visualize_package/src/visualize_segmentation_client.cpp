#include <visualize_package/visualize_client.hpp>

VisualizeClient::VisualizeClient(ros::NodeHandle &nh):
nh_(nh),
pnh_("~")
{
    set_parameter();
    hdf5_client_ = nh_.serviceClient<common_srvs::Hdf5OpenSegmentationService>(hdf5_open_acc_service_name_);
    visualize_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(visualize_service_name_);
}

void VisualizeClient::set_parameter()
{
    pnh_.getParam("visualize_client", param_list);
    hdf5_open_acc_service_name_ = "hdf5_open_segmentation_service";
    visualize_service_name_ = "visualize_cloud_service";
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
        common_srvs::Hdf5OpenSegmentationService hdf5_srv;
        hdf5_srv.request.index = index;
        hdf5_srv.request.hdf5_open_file_path = hdf5_open_file_path_;
        if (index == 0) {
            hdf5_srv.request.is_reload = 1;
        }
        else {
            hdf5_srv.request.is_reload = 0;
        }
        
        Util::message_show("index", index);
        Util::client_request(hdf5_client_, hdf5_srv, hdf5_open_acc_service_name_);
        common_srvs::VisualizeCloud visualize_srv;
        common_msgs::CloudData cloud_data = hdf5_srv.response.cloud_data;
        if (crop_) {
            pcl::PointCloud<pcl::PointXYZL> pcl_label_data = UtilMsgData::cloudmsg_to_pclLabel(cloud_data);
            pcl_label_data = cloud_process_.cropbox_segmenter(pcl_label_data);
            cloud_data = UtilMsgData::pclLabel_to_cloudmsg(pcl_label_data);
        }
        visualize_srv.request.cloud_data_list.push_back(cloud_data);
        visualize_srv.request.topic_name_list.push_back("index_" + std::to_string(index));
        Util::client_request(visualize_client_, visualize_srv, visualize_service_name_);
        if (index >= hdf5_srv.response.data_size - 1) {
            break;
        }
        index++;
    }
}

