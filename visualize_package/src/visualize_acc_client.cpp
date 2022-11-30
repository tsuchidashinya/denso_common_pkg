#include <visualize_package/visualize_client.hpp>

VisualizeClient::VisualizeClient(ros::NodeHandle &nh):
nh_(nh),
pnh_("~")
{
    set_parameter();
    hdf5_client_ = nh_.serviceClient<common_srvs::Hdf5OpenService>(hdf5_open_service_name_);
    visualize_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(visualize_service_name_);
    vis_img_client_ = nh_.serviceClient<common_srvs::VisualizeImage>(vis_img_service_name_);
}

void VisualizeClient::set_parameter()
{
    pnh_.getParam("visualize_client", param_list);
    hdf5_open_service_name_ = static_cast<std::string>(param_list["hdf5_open_service_name"]);
    Util::message_show("hdf5_open_service", hdf5_open_service_name_);
    visualize_service_name_ = static_cast<std::string>(param_list["visualize_service_name"]);
    vis_img_service_name_ = static_cast<std::string>(param_list["visualize_image_service_name"]);
}

void VisualizeClient::main()
{
    int data_size;
    nh_.getParam("hdf5_data_size", data_size);
    std::vector<common_msgs::CloudData> cloud_list;
    std::vector<sensor_msgs::Image> image_list;
    std::vector<std::string> topic_list;
    for (int i = 1; i <= data_size; i++) {
        common_srvs::Hdf5OpenService hdf5_srv;
        hdf5_srv.request.index = i;
        Util::client_request(hdf5_client_, hdf5_srv, hdf5_open_service_name_);
        cloud_list.push_back(hdf5_srv.response.cloud_data);
        topic_list.push_back("index_" + std::to_string(i));
        image_list.push_back(hdf5_srv.response.image);
    }
    common_srvs::VisualizeCloud visualize_srv;
    visualize_srv.request.cloud_data_list = cloud_list;
    visualize_srv.request.topic_name_list = topic_list;
    Util::client_request(visualize_client_, visualize_srv, visualize_service_name_);
    common_srvs::VisualizeImage visualize_img_srv;
    visualize_img_srv.request.image_list = image_list;
    visualize_img_srv.request.topic_name_list = topic_list;
    Util::client_request(vis_img_client_, visualize_img_srv, visualize_service_name_);
}

