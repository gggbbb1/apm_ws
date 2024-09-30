#pragma once
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

namespace gazebo
{
    class GimbalControllerPluginPrivate;

    class GimbalControllerPlugin : public ModelPlugin
    {
    public:
        GimbalControllerPlugin();
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void Init();

    private:
        void OnRequestMsg(ConstRequestPtr &_msg);
        std::unique_ptr<GimbalControllerPluginPrivate> dataPtr;
        void OnUpdate();
    };
}