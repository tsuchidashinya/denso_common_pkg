#include <visualize_package/visualize_client.hpp>

VisualizeClient::VisualizeClient(ros::NodeHandle &nh):
nh_(nh),
pnh_("~")
{
    set_parameter();
    hdf5_client_ = nh_.serviceClient<common_srvs::Hdf5OpenSegmentationService>(hdf5_open_service_name_);
    visualize_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(visualize_service_name_);
}

void VisualizeClient::set_parameter()
{
    pnh_.getParam("visualize_client", param_list);
    hdf5_open_service_name_ = static_cast<std::string>(param_list["hdf5_open_service_name"]);
    Util::message_show("hdf5_open_service", hdf5_open_service_name_);
    visualize_service_name_ = static_cast<std::string>(param_list["visualize_service_name"]);
}

void VisualizeClient::main()
{
    int index = 1;
    while (1) {
        common_srvs::Hdf5OpenSegmentationService hdf5_srv;
        hdf5_srv.request.index = index;
        Util::message_show("index", index);
        Util::client_request(hdf5_client_, hdf5_srv, hdf5_open_service_name_);
        common_srvs::VisualizeCloud visualize_srv;
        visualize_srv.request.cloud_data_list.push_back(hdf5_srv.response.cloud_data);
        visualize_srv.request.topic_name_list.push_back("index_" + std::to_string(index));
        Util::client_request(visualize_client_, visualize_srv, visualize_service_name_);
        if (index >= hdf5_srv.response.data_size) {
            break;
        }
        index++;
    }
}

