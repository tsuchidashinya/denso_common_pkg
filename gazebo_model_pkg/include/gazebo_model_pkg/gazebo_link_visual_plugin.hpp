#pragma once
#include <string>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/advertise_options.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#define protected public
#include <gazebo/physics/physics.hh>
#undef protected
#include <gazebo/physics/Link.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/TransportIface.hh>
#include <common_srvs/SetLinkVisualProperties.h>
#include <common_srvs/GetVisualNames.h>


namespace gazebo
{
    class GazeboLinkVisualPlugin : public WorldPlugin
    {
        public: GazeboLinkVisualPlugin();
        public: virtual ~GazeboLinkVisualPlugin();
        protected: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
        private: physics::WorldPtr world_;
        private: boost::mutex lock_;
        private: transport::NodePtr gzNode;
        private: transport::PublisherPtr visualPub;
        private: boost::thread callback_queue_thread_;
        private: ros::NodeHandle* rosnode_;
        private: ros::CallbackQueue queue_;
        private: ros::ServiceServer set_link_visual_, get_visual_name_;
        private: bool SetLinkVisualCallback(common_srvs::SetLinkVisualProperties::Request&, common_srvs::SetLinkVisualProperties::Response&);
        private: bool GetVisualNameCallback(common_srvs::GetVisualNames::Request&, common_srvs::GetVisualNames::Response&);
        private: void QueueThread();
    };
}