#ifndef ROADROLLER_CONTROL__ROADROLLER_CONTROL_PLUGIN_HPP
#define ROADROLLER_CONTROL__ROADROLLER_CONTROL_PLUGIN_HPP

#include <ros/ros.h>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <ackermann_msgs/AckermannDrive.h>

namespace ROADROLLER_CONTROL
{

  class RoadrollerControlPlugin : public gazebo::ModelPlugin
  {
  public:
    RoadrollerControlPlugin();

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

  private:
    std::string robot_namespace_;

    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;
    gazebo::transport::NodePtr gznode_;

    gazebo::physics::LinkPtr chassis_link_;

    gazebo::physics::JointPtr rear_left_wheel_joint_;
    gazebo::physics::JointPtr rear_inner_left_wheel_joint_;
    gazebo::physics::JointPtr rear_inner_right_wheel_joint_;
    gazebo::physics::JointPtr rear_right_wheel_joint_;
    gazebo::physics::JointPtr front_left_wheel_joint_;
    gazebo::physics::JointPtr front_center_wheel_joint_;
    gazebo::physics::JointPtr front_right_wheel_joint_;
    gazebo::physics::JointPtr front_left_wheel_steering_joint_;
    gazebo::physics::JointPtr front_center_wheel_steering_joint_;
    gazebo::physics::JointPtr front_right_wheel_steering_joint_;

    // steering PIDs
    gazebo::common::PID front_left_wheel_steering_pid_;
    gazebo::common::PID front_center_wheel_steering_pid_;
    gazebo::common::PID front_right_wheel_steering_pid_;

    // traction PIDs
    gazebo::common::PID rear_left_wheel_axle_pid_;
    gazebo::common::PID rear_inner_left_wheel_axle_pid_;
    gazebo::common::PID rear_inner_right_wheel_axle_pid_;
    gazebo::common::PID rear_right_wheel_axle_pid_;

    double rear_left_wheel_radius_;
    double rear_inner_left_wheel_radius_;
    double rear_inner_right_wheel_radius_;
    double rear_right_wheel_radius_;
    double front_left_wheel_radius_;
    double front_center_wheel_radius_;
    double front_right_wheel_radius_;

    double front_track_width_;
    double rear_track_width_;
    double wheel_base_length_;

    double max_steer_;
    double max_speed_;
    double max_torque_;
    double front_brake_torque_;
    double back_brake_torque_;
    double chassis_aero_force_gain_;
    double start_braking_threshold_;
    double vel_error_max_braking_;

    ros::Subscriber control_sub_;
    ros::Publisher odometry_pub_;

    gazebo::common::Time last_sim_time_;

    ackermann_msgs::AckermannDrive ackermann_cmd_;
    ackermann_msgs::AckermannDrive ackermann_cmd_old_;

    std::mutex mutex_;

    gazebo::event::ConnectionPtr update_connection_;
    void Update();

    void publishOdometry();

    void controlCallback(const ackermann_msgs::AckermannDrive & msg);

    double collisionRadius(gazebo::physics::CollisionPtr coll) const;

  };

}

#endif