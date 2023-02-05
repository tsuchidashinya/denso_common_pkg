#include <gazebo_model_pkg/gazebo_link_visual_plugin.hpp>

namespace gazebo
{
    // GZ_REGISTER_WORLD_PLUGIN(GazeboLinkVisualPlugin)
    GazeboLinkVisualPlugin::GazeboLinkVisualPlugin()
    {

    }

    GazeboLinkVisualPlugin::~GazeboLinkVisualPlugin()
    {
        this->gzNode->Fini();
        this->rosnode_->shutdown();
        this->callback_queue_thread_.join();
        delete this->rosnode_;
    }

    void GazeboLinkVisualPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
        this->world_ = _parent;
        this->rosnode_ = new ros::NodeHandle("~");

        this->gzNode = transport::NodePtr(new transport::Node());
        this->gzNode->Init();
        this->visualPub = this->gzNode->Advertise<msgs::Visual>("~/visual");
        ros::AdvertiseServiceOptions set_col_aso =
        ros::AdvertiseServiceOptions::create<common_srvs::SetLinkVisualProperties>(
        "set_link_visual", boost::bind(&GazeboLinkVisualPlugin::SetLinkVisualCallback,
        this, _1, _2), ros::VoidPtr(), &this->queue_);
        this->set_link_visual_ = this->rosnode_->advertiseService(set_col_aso);
         this->callback_queue_thread_ =
            boost::thread(boost::bind(&GazeboLinkVisualPlugin::QueueThread, this));
    }

    void GazeboLinkVisualPlugin::QueueThread()
    {
        static const double timeout = 0.01;

        while (this->rosnode_->ok())
        {
            this->queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    bool GazeboLinkVisualPlugin::SetLinkVisualCallback(common_srvs::SetLinkVisualProperties::Request &req,
                                               common_srvs::SetLinkVisualProperties::Response &res)
    {
        boost::lock_guard<boost::mutex> lock(this->lock_);

        //gets the visual message
        #if GAZEBO_MAJOR_VERSION >= 8
        physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(world_->EntityByName(req.link_parent_name));
        #else
        physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(world_->GetEntity(req.link_parent_name));
        #endif

        msgs::Visual visualMsg = link->GetVisualMessage(req.link_visual_name);
        visualMsg.set_name(link->GetScopedName());

        //starts a new material, in case the object doesn't have one
        if ((!visualMsg.has_material()) || visualMsg.mutable_material() == NULL) {
            msgs::Material *materialMsg = new msgs::Material;
            visualMsg.set_allocated_material(materialMsg);
        }

        
        // ignition::math::Vector3 ambient(req.ambient.r, req.ambient.g, req.ambient.b);
        
        // Set color
        // common::Color ambient(req.ambient.r, req.ambient.g, req.ambient.b, req.ambient.a);
        // common::Color diffuse(req.diffuse.r, req.diffuse.g, req.diffuse.b, req.diffuse.a);
        msgs::Color ambient, diffuse;
        ambient.set_a(req.ambient.a);
        ambient.set_r(req.ambient.r);
        ambient.set_g(req.ambient.g);
        ambient.set_b(req.ambient.b);

        diffuse.set_a(req.diffuse.a);
        diffuse.set_r(req.diffuse.r);
        diffuse.set_g(req.diffuse.g);
        diffuse.set_b(req.diffuse.b);
        

        // msgs::Color *colorMsg = new msgs::Color(msgs::Convert(ambient));
        // msgs::Color *diffuseMsg = new msgs::Color(msgs::Convert(diffuse));

        msgs::Color *colorMsg = new msgs::Color(ambient);
        msgs::Color *diffuseMsg = new msgs::Color(diffuse);
        

        //add color to material
        msgs::Material *materialMsg = visualMsg.mutable_material();
        if (materialMsg->has_ambient())
        {
            materialMsg->clear_ambient();
        }
        materialMsg->set_allocated_ambient(colorMsg);
        if (materialMsg->has_diffuse())
        {
            materialMsg->clear_diffuse();
        }
        materialMsg->set_allocated_diffuse(diffuseMsg);

        //publish message
        this->visualPub->Publish(visualMsg);

        res.success = true;
        return true;
    }
}