#include <gazebo_model_pkg/gazebo_model_move.hpp>

GazeboMoveServer::GazeboMoveServer(ros::NodeHandle nh) : nh_(nh), pnh_("~")
                                                    
{
    set_parameter();
    gazebo_pub_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    gazebo_light_client_ = nh_.serviceClient<gazebo_msgs::SetLightProperties>(light_service_name_);
    gazebo_transport_pub_ = gzNode_.Advertise<gazebo::msgs::Visual>("~/visual");
}

void GazeboMoveServer::set_link_visual(gazebo::msgs::Visual visual)
{
   
    gazebo_transport_pub_->Publish(visual);
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
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
}

