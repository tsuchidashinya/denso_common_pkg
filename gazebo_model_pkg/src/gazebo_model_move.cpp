#include <gazebo_model_pkg/gazebo_model_move.hpp>

GazeboMoveServer::GazeboMoveServer(ros::NodeHandle nh) : nh_(nh), pnh_("~")
                                                    
{
    set_parameter();
    gazebo_pub_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    gazebo_light_client_ = nh_.serviceClient<gazebo_msgs::SetLightProperties>(light_service_name_);
    gazebo_link_visual_client_ = nh_.serviceClient<common_srvs::SetLinkVisualProperties>(link_visual_service_name_);
    gazebo_visual_client_ = nh_.serviceClient<common_srvs::GetVisualNames>(visualname_service_name_);
    gazebo_model_proporties_client_ = nh_.serviceClient<gazebo_msgs::GetModelProperties>(model_proporties_service_name_);
}

void GazeboMoveServer::set_link_visual(common_srvs::SetLinkVisualPropertiesRequest visual)
{
    gazebo_msgs::GetModelProperties model_proporties_srv;
    model_proporties_srv.request.model_name = visual.link_name;
    Util::client_request(gazebo_model_proporties_client_, model_proporties_srv, model_proporties_service_name_);
    std::vector<std::string> link_names;
    for (int i = 0; i < model_proporties_srv.response.body_names.size(); i++) {
        std::string link_name = visual.link_name + "::" + model_proporties_srv.response.body_names[i];
        link_names.push_back(link_name);
    }
    common_srvs::GetVisualNames visual_name_srv;
    visual_name_srv.request.link_names = link_names;
    Util::client_request(gazebo_visual_client_, visual_name_srv, visualname_service_name_);
    auto index_choise = Util::random_int_static(0, visual_name_srv.response.link_parent_names.size() - 1);
    visual.link_parent_name = visual_name_srv.response.link_parent_names[index_choise];
    visual.link_visual_name = visual_name_srv.response.link_visual_names[index_choise];
    common_srvs::SetLinkVisualProperties link_visual_proporties_srv;
    link_visual_proporties_srv.request = visual;
    Util::client_request(gazebo_link_visual_client_, link_visual_proporties_srv, link_visual_service_name_);
}

void GazeboMoveServer::set_multi_gazebo_model(std::vector<common_msgs::ObjectInfo> multi_object_info)
{
    for (int i = 0; i < multi_object_info.size(); i++)
    {
        gazebo_msgs::ModelState model_state = UtilMsgData::make_gazebo_model_state(multi_object_info[i]);
        for (int j = 0; j < 5; j++)
        {
            gazebo_pub_.publish(model_state);
            ros::Duration(0.001).sleep();
        }
    }
}

void GazeboMoveServer::set_light(gazebo_msgs::SetLightPropertiesRequest light_request)
{
    gazebo_msgs::SetLightProperties light_srv;
    light_srv.request = light_request;
    Util::client_request(gazebo_light_client_, light_srv, light_service_name_);
}

void GazeboMoveServer::set_gazebo_model(common_msgs::ObjectInfo object_info)
{
    for (int j = 0; j < 5; j++)
    {
        gazebo_msgs::ModelState model_state = UtilMsgData::make_gazebo_model_state(object_info);
        gazebo_pub_.publish(model_state);
        ros::Duration(0.01).sleep();
    }
}


void GazeboMoveServer::set_parameter()
{
    pnh_.getParam("common_parameter", param_list);
    light_service_name_ = "/gazebo/set_light_properties";
    visualname_service_name_ = "/gazebo/get_visual_names";
    link_visual_service_name_ = "/gazebo/set_link_visual";
    model_proporties_service_name_ = "/gazebo/get_model_properties";
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
}

