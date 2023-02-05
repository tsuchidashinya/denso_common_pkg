/**
 * @file gazebo_model_server.hpp
 * @author tsuchidashinya (tsuchida.shinya413@mail.kyutech.jp)
 * @brief
 * @version 0.1
 * @date 2022-09-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once
#include <util_package/util.hpp>
#include "decide_object_position.hpp"
#include <util_package/util_msg_data.hpp>



class GazeboMoveServer
{
public:
    GazeboMoveServer(ros::NodeHandle);
   
    void set_multi_gazebo_model(std::vector<common_msgs::ObjectInfo>);
    void set_gazebo_model(common_msgs::ObjectInfo);
    void set_light(gazebo_msgs::SetLightPropertiesRequest);
    void set_link_visual(common_srvs::SetLinkVisualPropertiesRequest);
    common_srvs::GetVisualNamesResponse get_visual_name(std::string);
    void set_parameter();
private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher gazebo_pub_;
    ros::ServiceClient gazebo_light_client_, gazebo_link_visual_client_, gazebo_visual_client_, gazebo_model_proporties_client_;
    std::string world_frame_;
    std::string light_service_name_, link_visual_service_name_, visualname_service_name_, model_proporties_service_name_;
    XmlRpc::XmlRpcValue param_list;
    TfFunction tf_func_;
};
