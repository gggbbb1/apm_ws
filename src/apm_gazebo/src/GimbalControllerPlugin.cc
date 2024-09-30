
#include <iostream>
#include <vector>
#include "gazebo/transport/transport.hh"
#include "GimbalControllerPlugin.hh"
#include <gazebo/sensors/sensors.hh>
#include <map>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GimbalControllerPlugin)

class gazebo::GimbalControllerPluginPrivate
{
public:
  common::PID roll_pid;
  common::PID pitch_pid;
  common::PID yaw_pid;

  double roll_command=0.0;
  double pitch_command=0.0;
  double yaw_command=0.0;

  double yaw_filter=0.0;

  physics::JointPtr roll_joint;
  physics::JointPtr pitch_joint;
  physics::JointPtr yaw_joint;
  physics::ModelPtr model;

  std::vector<event::ConnectionPtr> connections;
  common::Time lastUpdateTime;
  sensors::ImuSensorPtr roll_imu_sensor;
  sensors::ImuSensorPtr pitch_imu_sensor;

  transport::SubscriberPtr sub;
  transport::NodePtr node;
};

GimbalControllerPlugin::GimbalControllerPlugin()
    : dataPtr(new GimbalControllerPluginPrivate)
{
  this->dataPtr->pitch_pid.Init(1.0, 0.1, 0.05, 0.1, -0.1, 1.0, -1.0);
  this->dataPtr->roll_pid.Init(2.0, 0.2, 0.1, 0.1, -0.1, 1.0, -1.0);
  this->dataPtr->yaw_pid.Init(2.0, 0.1, 0.8, 0.02, -0.02, 1.0, -1.0);
}

void GimbalControllerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzmsg << "Gimbal controller 0.3" << std::endl;
  this->dataPtr->model = _model;

  this->dataPtr->roll_joint = this->dataPtr->model->GetJoint("my_gimbal_3d::roll");
  gzwarn << "roll joint name: " << this->dataPtr->roll_joint->GetName() << std::endl;

  this->dataPtr->pitch_joint = this->dataPtr->model->GetJoint("my_gimbal_3d::tilt");
  gzwarn << "tilt joint name: " << this->dataPtr->pitch_joint->GetName() << std::endl;

  this->dataPtr->yaw_joint = this->dataPtr->model->GetJoint("my_gimbal_3d::pan");
  gzwarn << "yaw joint name: " << this->dataPtr->yaw_joint->GetName() << std::endl;

  this->dataPtr->roll_imu_sensor = std::static_pointer_cast<sensors::ImuSensor>(
      sensors::SensorManager::Instance()->GetSensor(_model->SensorScopedName("camera_imu")[0]));

  this->dataPtr->pitch_imu_sensor = std::static_pointer_cast<sensors::ImuSensor>(
      sensors::SensorManager::Instance()->GetSensor(_model->SensorScopedName("pitch_camera_imu")[0]));

}

void GimbalControllerPlugin::Init()
{
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->model->GetWorld()->Name());

  this->dataPtr->lastUpdateTime =
      this->dataPtr->model->GetWorld()->SimTime();

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&GimbalControllerPlugin::OnUpdate, this)));

  std::string topic = "~/" + this->dataPtr->model->GetName() + "/command_request";
  this->dataPtr->sub = this->dataPtr->node->Subscribe(topic,
                                                      &GimbalControllerPlugin::OnRequestMsg, this);
}

void GimbalControllerPlugin::OnRequestMsg(ConstRequestPtr &_msg)
{
  /*
  msg API
  -------
  int id:
  string request: roll, pitch(tilt), yaw (pan)
  string data -> mode: pwm , degree, rad
  double dbl_data: relative to select mode :
  pwm    [1000(-120d)  :  2000(+120d)]
  degree [-180         :  +180]
  rad    [-pi          :  +pi]

  command example:
  ----------------
  gz topic -p "/gazebo/default/stand_with_gimbal/command_request"  \
    "gazebo.msgs.Request" -m 'id: 1, request: "yaw", data: "degree", dbl_data: 90.0 '
  */
  double rad = 0;
  auto mode = _msg->data();
  std::transform(mode.begin(), mode.end(), mode.begin(), ::toupper);
  auto cmd = _msg->dbl_data();
  if (mode == "PWM")
  {
    auto deg = ((cmd - 1500) / 500) * 120;
    rad = 3.14 * deg / 180;
  }
  else if (mode == "DEGREE")
  {
    rad = 3.14 * cmd / 180;
  }
  else
  {
    rad = cmd;
  }

  // gzwarn << "command: " << _msg->request() << ":" << rad << std::endl;
 
  if (_msg->request()=="roll"){
    this->dataPtr->roll_command=rad;
  }
  else if (_msg->request()=="pitch")
  {
    this->dataPtr->pitch_command=-rad;
  }
  else if (_msg->request()=="yaw")
  {
    this->dataPtr->yaw_command=-rad;
  }
  
}

void GimbalControllerPlugin::OnUpdate()
{
  if (!this->dataPtr->roll_joint)
  {
    gzerr << "joitn not found" << std::endl;
    return;
  }
  common::Time time = this->dataPtr->model->GetWorld()->SimTime();
  if (time < this->dataPtr->lastUpdateTime)
  {
    this->dataPtr->lastUpdateTime = time;
    return;
  }

  double yaw_angle = this->dataPtr->yaw_joint->Position(0);
  this->dataPtr->yaw_filter = 0.005*yaw_angle + 0.995*this->dataPtr->yaw_filter;

  double dt = (time - this->dataPtr->lastUpdateTime).Double();
  if (dt<0.01)
  {
    return;
  }
  this->dataPtr->lastUpdateTime = time;

  auto roll_camera_imu = this->dataPtr->roll_imu_sensor->Orientation().Euler().X();
  auto pitch_camera_imu = this->dataPtr->pitch_imu_sensor->Orientation().Euler().X();

  double roll_error = roll_camera_imu - this->dataPtr->roll_command;
  double roll_force = this->dataPtr->roll_pid.Update(roll_error, dt);
  this->dataPtr->roll_joint->SetForce(0, roll_force);

  double pitch_error = pitch_camera_imu - this->dataPtr->pitch_command;
  double pitch_force = this->dataPtr->pitch_pid.Update(pitch_error, dt);
  this->dataPtr->pitch_joint->SetForce(0, pitch_force);

  
  double yaw_error = this->dataPtr->yaw_filter - this->dataPtr->yaw_command;
  double yaw_force = this->dataPtr->yaw_pid.Update(yaw_error, dt);
  this->dataPtr->yaw_joint->SetForce(0, yaw_force);

  // gzmsg << "roll_error: " << roll_error << "   force: " << roll_force << std::endl;
  // gzmsg << "pitch_error: " << pitch_error << std::endl;
  // gzmsg << "yaw_error: " << yaw_error << std::endl;

  
}