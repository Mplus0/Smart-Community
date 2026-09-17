#include <cmath>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <boost/bind/bind.hpp>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>

namespace gazebo
{

// Planar velocity plugin for robotcar.
//
// /cmd_vel is defined at the base_footprint link origin.  Unlike the stock
// gazebo_ros_planar_move implementation, this plugin deliberately does NOT use
// Model::SetLinearVel / Model::SetAngularVel, because model-level setters apply
// velocities to every link in the articulated model.  The wheel links must be
// left to Gazebo joint constraints and the wheel velocity controller.
//
// Gazebo / ODE stores linear velocity at a link's physical centre of gravity
// (CoG), while WorldPose() refers to the link frame origin.  Because the
// official chassis inertia has an offset CoG, a pure yaw command needs a small
// CoG translation so that the base_footprint origin itself stays fixed:
//
//   v_cog = v_base + omega x r_(base->cog)
//
// Planar body-frame form:
//   v_cog_x = vx - wz * r_y
//   v_cog_y = vy + wz * r_x
//
// The CoG offset is read directly from the final Gazebo link inertial, avoiding
// a duplicated hard-coded mass-distribution value in the Xacro.
class RobotcarPlanarMove : public ModelPlugin
{
public:
  RobotcarPlanarMove() = default;

  ~RobotcarPlanarMove() override
  {
    this->alive_ = false;
    this->queue_.clear();
    this->queue_.disable();

    if (this->rosnode_)
      this->rosnode_->shutdown();

    if (this->callback_queue_thread_.joinable())
      this->callback_queue_thread_.join();
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model_ = model;

    this->robot_namespace_ = "";
    if (sdf->HasElement("robotNamespace"))
      this->robot_namespace_ =
          sdf->GetElement("robotNamespace")->Get<std::string>();

    this->command_topic_ = "cmd_vel";
    if (sdf->HasElement("commandTopic"))
      this->command_topic_ =
          sdf->GetElement("commandTopic")->Get<std::string>();

    this->base_link_name_ = "base_footprint";
    if (sdf->HasElement("baseLink"))
      this->base_link_name_ =
          sdf->GetElement("baseLink")->Get<std::string>();

    this->cmd_timeout_ = 0.5;
    if (sdf->HasElement("cmdTimeout"))
      this->cmd_timeout_ = sdf->GetElement("cmdTimeout")->Get<double>();

    this->base_link_ = this->model_->GetLink(this->base_link_name_);
    if (!this->base_link_)
    {
      ROS_FATAL_STREAM_NAMED(
          "robotcar_planar_move",
          "Could not find Gazebo link [" << this->base_link_name_
          << "] in model [" << this->model_->GetName() << "].");
      return;
    }

    const physics::InertialPtr inertial = this->base_link_->GetInertial();
    if (!inertial)
    {
      ROS_FATAL_STREAM_NAMED(
          "robotcar_planar_move",
          "Gazebo link [" << this->base_link_name_
          << "] has no inertial; cannot compensate its CoG offset.");
      return;
    }

    this->base_to_cog_ = inertial->CoG();

    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM_NAMED(
          "robotcar_planar_move",
          "ROS is not initialized. Start Gazebo through gazebo_ros so that "
          "libgazebo_ros_api_plugin.so is loaded.");
      return;
    }

    this->rosnode_.reset(new ros::NodeHandle(this->robot_namespace_));

    ros::SubscribeOptions options =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            this->command_topic_, 1,
            boost::bind(&RobotcarPlanarMove::CmdVelCallback, this,
                        boost::placeholders::_1),
            ros::VoidPtr(), &this->queue_);
    this->cmd_sub_ = this->rosnode_->subscribe(options);

    this->last_cmd_received_time_ = ros::Time(0);
    this->alive_ = true;
    this->callback_queue_thread_ =
        std::thread(&RobotcarPlanarMove::QueueThread, this);

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RobotcarPlanarMove::Update, this));

    ROS_INFO_STREAM_NAMED(
        "robotcar_planar_move",
        "Robotcar base-link planar move loaded: topic="
        << this->command_topic_
        << ", baseLink=" << this->base_link_name_
        << ", cmdTimeout=" << this->cmd_timeout_
        << ", link CoG xyz=[" << this->base_to_cog_.X()
        << ", " << this->base_to_cog_.Y()
        << ", " << this->base_to_cog_.Z() << "] m");
  }

private:
  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->last_cmd_received_time_ = ros::Time::now();
    this->vx_ = msg->linear.x;
    this->vy_ = msg->linear.y;
    this->wz_ = msg->angular.z;
  }

  void Update()
  {
    double vx;
    double vy;
    double wz;
    ros::Time last_cmd_received_time;

    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      vx = this->vx_;
      vy = this->vy_;
      wz = this->wz_;
      last_cmd_received_time = this->last_cmd_received_time_;
    }

    if (this->cmd_timeout_ >= 0.0)
    {
      const ros::Time now = ros::Time::now();
      if (last_cmd_received_time.isZero() ||
          (now - last_cmd_received_time).toSec() > this->cmd_timeout_)
      {
        vx = 0.0;
        vy = 0.0;
        wz = 0.0;
      }
    }

    if (!this->base_link_)
      return;

    // Desired velocity is at the base_footprint frame origin.  ODE's link
    // linear velocity is the CoG velocity, so compensate for the inertial
    // offset before writing the base link state.
    const double cog_vx_body = vx - wz * this->base_to_cog_.Y();
    const double cog_vy_body = vy + wz * this->base_to_cog_.X();

    const double yaw = this->base_link_->WorldPose().Rot().Yaw();
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    const double cog_vx_world = cog_vx_body * cy - cog_vy_body * sy;
    const double cog_vy_world = cog_vy_body * cy + cog_vx_body * sy;

    // Only the canonical chassis link is driven here.  Wheel-link motion is
    // left to the revolute joints and, when enabled, the wheel controller.
    this->base_link_->SetLinearVel(
        ignition::math::Vector3d(cog_vx_world, cog_vy_world, 0.0));
    this->base_link_->SetAngularVel(
        ignition::math::Vector3d(0.0, 0.0, wz));
  }

  void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->alive_ && this->rosnode_ && this->rosnode_->ok())
      this->queue_.callAvailable(ros::WallDuration(timeout));
  }

private:
  physics::ModelPtr model_;
  physics::LinkPtr base_link_;
  event::ConnectionPtr update_connection_;

  std::unique_ptr<ros::NodeHandle> rosnode_;
  ros::Subscriber cmd_sub_;
  ros::CallbackQueue queue_;
  std::thread callback_queue_thread_;
  std::mutex mutex_;

  std::atomic<bool> alive_{false};
  std::string robot_namespace_;
  std::string command_topic_;
  std::string base_link_name_;

  double cmd_timeout_{0.5};
  ignition::math::Vector3d base_to_cog_{0.0, 0.0, 0.0};

  double vx_{0.0};
  double vy_{0.0};
  double wz_{0.0};
  ros::Time last_cmd_received_time_;
};

GZ_REGISTER_MODEL_PLUGIN(RobotcarPlanarMove)

}  // namespace gazebo
