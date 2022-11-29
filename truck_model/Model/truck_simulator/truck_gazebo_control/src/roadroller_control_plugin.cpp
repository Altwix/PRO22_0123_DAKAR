#include "roadroller_control/roadroller_control_plugin.hpp"

#include <gazebo/physics/physics.hh>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

namespace ROADROLLER_CONTROL
{

  RoadrollerControlPlugin::RoadrollerControlPlugin()
      : rear_left_wheel_radius_{0},
        rear_inner_left_wheel_radius_{0},
        rear_inner_right_wheel_radius_{0},
        rear_right_wheel_radius_{0},
        front_left_wheel_radius_{0},
        front_center_wheel_radius_{0},
        front_right_wheel_radius_{0},
        last_sim_time_{0},
        robot_namespace_{""}
  {
  }

  void RoadrollerControlPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    ROS_INFO("Loading plugin!");

    model_ = model;
    world_ = model_->GetWorld();
    auto physicsEngine = world_->Physics(); //->GetPhysicsEngine();
    physicsEngine->SetParam("friction_model", std::string{"cone_model"});

    // object to communicate with ROS
    gznode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gznode_->Init();

    if (sdf->HasElement("robotNamespace"))
    {
      ROS_INFO_STREAM("tiene namespace y es " + sdf->GetElement("robotNamespace")->Get<std::string>());
      robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
      // robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }
    // ROS node creation
    ros::NodeHandle nh(this->robot_namespace_);

    // subscriptions to ROS topics
    control_sub_ = nh.subscribe(
        "/roadroller_cmd", 10, &RoadrollerControlPlugin::controlCallback, this);

    // Find joints and links
    auto findLink = [&](std::string const &link_name) {
      auto full_link_name = model_->GetName() + "::" + sdf->Get<std::string>(link_name);
      auto link = model_->GetLink(full_link_name);
      if (!link)
        std::cerr << "could not find link: " << full_link_name << "\n";
      return link;
    };

    auto findJoint = [&](std::string const &joint_name) {
      auto full_joint_name = model_->GetName() + "::" + sdf->Get<std::string>(joint_name);
      auto joint = model_->GetJoint(full_joint_name);
      if (!joint)
        std::cerr << "could not find joint: " << full_joint_name << "\n";
      return joint;
    };

    chassis_link_ = findLink("chassis");

    rear_left_wheel_joint_ = findJoint("rear_left_wheel");
    rear_inner_left_wheel_joint_ = findJoint("rear_inner_left_wheel");
    rear_inner_right_wheel_joint_ = findJoint("rear_inner_right_wheel");
    rear_right_wheel_joint_ = findJoint("rear_right_wheel");
    front_left_wheel_joint_ = findJoint("front_left_wheel");
    front_center_wheel_joint_ = findJoint("front_center_wheel");
    front_right_wheel_joint_ = findJoint("front_right_wheel");
    front_left_wheel_steering_joint_ = findJoint("front_left_steering_joint");
    front_center_wheel_steering_joint_ = findJoint("front_center_steering_joint");
    front_right_wheel_steering_joint_ = findJoint("front_right_steering_joint");

    // Read parameters
    auto findParameter = [&](std::string const &param_name, double default_value) {
      if (sdf->HasElement(param_name))
        return sdf->Get<double>(param_name);
      else
        return default_value;
    };

    max_steer_ = findParameter("max_steer", 0.785398);
    max_speed_ = findParameter("max_speed", 10.0);
    max_torque_ = findParameter("max_torque", 57.0);
    front_brake_torque_ = findParameter("front_brake_torque", 500.0);
    back_brake_torque_ = findParameter("back_brake_torque", 500.0);
    chassis_aero_force_gain_ = findParameter("chassis_aero_force_gain", 1.0);
    start_braking_threshold_ = findParameter("start_braking_threshold", 0.3);
    vel_error_max_braking_ = findParameter("vel_error_max_braking", 1.0);

    // seting steering PID gains

    front_left_wheel_steering_pid_.SetPGain(findParameter("wheel_steering_p_gain", 0.0));
    front_left_wheel_steering_pid_.SetIGain(findParameter("wheel_steering_i_gain", 0.0));
    front_left_wheel_steering_pid_.SetDGain(findParameter("wheel_steering_d_gain", 0.0));

    front_center_wheel_steering_pid_.SetPGain(findParameter("wheel_steering_p_gain", 0.0));
    front_center_wheel_steering_pid_.SetIGain(findParameter("wheel_steering_i_gain", 0.0));
    front_center_wheel_steering_pid_.SetDGain(findParameter("wheel_steering_d_gain", 0.0));

    front_right_wheel_steering_pid_.SetPGain(findParameter("wheel_steering_p_gain", 0.0));
    front_right_wheel_steering_pid_.SetIGain(findParameter("wheel_steering_i_gain", 0.0));
    front_right_wheel_steering_pid_.SetDGain(findParameter("wheel_steering_d_gain", 0.0));

    // seting axle PID gains

    rear_left_wheel_axle_pid_.SetPGain(findParameter("wheel_axle_p_gain", 0.0));
    rear_left_wheel_axle_pid_.SetIGain(findParameter("wheel_axle_i_gain", 0.0));
    rear_left_wheel_axle_pid_.SetDGain(findParameter("wheel_axle_d_gain", 0.0));

    rear_inner_left_wheel_axle_pid_.SetPGain(findParameter("wheel_axle_p_gain", 0.0));
    rear_inner_left_wheel_axle_pid_.SetIGain(findParameter("wheel_axle_i_gain", 0.0));
    rear_inner_left_wheel_axle_pid_.SetDGain(findParameter("wheel_axle_d_gain", 0.0));

    rear_inner_right_wheel_axle_pid_.SetPGain(findParameter("wheel_axle_p_gain", 0.0));
    rear_inner_right_wheel_axle_pid_.SetIGain(findParameter("wheel_axle_i_gain", 0.0));
    rear_inner_right_wheel_axle_pid_.SetDGain(findParameter("wheel_axle_d_gain", 0.0));

    rear_right_wheel_axle_pid_.SetPGain(findParameter("wheel_axle_p_gain", 0.0));
    rear_right_wheel_axle_pid_.SetIGain(findParameter("wheel_axle_i_gain", 0.0));
    rear_right_wheel_axle_pid_.SetDGain(findParameter("wheel_axle_d_gain", 0.0));

    /* front_left_wheel_steering_pid_.SetPGain(findParameter("front_left_wheel_steering_p_gain", 0.0));
    front_left_wheel_steering_pid_.SetIGain(findParameter("front_left_wheel_steering_i_gain", 0.0));
    front_left_wheel_steering_pid_.SetDGain(findParameter("front_left_wheel_steering_d_gain", 0.0));

    front_center_wheel_steering_pid_.SetPGain(findParameter("front_center_wheel_steering_p_gain", 0.0));
    front_center_wheel_steering_pid_.SetIGain(findParameter("front_center_wheel_steering_i_gain", 0.0));
    front_center_wheel_steering_pid_.SetDGain(findParameter("front_center_wheel_steering_d_gain", 0.0));

    front_right_wheel_steering_pid_.SetPGain(findParameter("front_right_wheel_steering_p_gain", 0.0));
    front_right_wheel_steering_pid_.SetIGain(findParameter("front_right_wheel_steering_i_gain", 0.0));
    front_right_wheel_steering_pid_.SetDGain(findParameter("front_right_wheel_steering_d_gain", 0.0)); */

    front_left_wheel_steering_pid_.SetCmdMin(-5000);
    front_left_wheel_steering_pid_.SetCmdMax(5000);
    front_center_wheel_steering_pid_.SetCmdMin(-5000);
    front_center_wheel_steering_pid_.SetCmdMax(5000);
    front_right_wheel_steering_pid_.SetCmdMin(-5000);
    front_right_wheel_steering_pid_.SetCmdMax(5000);

    // Determine physical properties
    auto id = unsigned{0};
    rear_left_wheel_radius_ = collisionRadius(rear_left_wheel_joint_->GetChild()->GetCollision(id));
    rear_inner_left_wheel_radius_ = collisionRadius(rear_inner_left_wheel_joint_->GetChild()->GetCollision(id));
    rear_inner_right_wheel_radius_ = collisionRadius(rear_inner_right_wheel_joint_->GetChild()->GetCollision(id));
    rear_right_wheel_radius_ = collisionRadius(rear_right_wheel_joint_->GetChild()->GetCollision(id));
    front_left_wheel_radius_ = collisionRadius(front_left_wheel_joint_->GetChild()->GetCollision(id));
    front_center_wheel_radius_ = collisionRadius(front_center_wheel_joint_->GetChild()->GetCollision(id));
    front_right_wheel_radius_ = collisionRadius(front_right_wheel_joint_->GetChild()->GetCollision(id));

    ROS_INFO_STREAM("Radii found:" << rear_left_wheel_radius_ << " " << rear_inner_left_wheel_radius_ << " " << rear_inner_right_wheel_radius_ << " " << rear_right_wheel_radius_ << " " << front_left_wheel_radius_ << " " << front_center_wheel_radius_ << " " << front_right_wheel_radius_);

    // Compute wheelbase, frontTrackWidth, and rearTrackWidth
    //  first compute the positions of the 4 wheel centers
    //  again assumes wheel link is child of joint and has only one collision
    auto rear_left_center_pos = rear_left_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();  //GetWorldPose().Ign().Pos();
    auto rear_inner_left_center_pos = rear_inner_left_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();  //GetWorldPose().Ign().Pos();
    auto rear_inner_right_center_pos = rear_inner_right_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();  //GetWorldPose().Ign().Pos();
    auto rear_right_center_pos = rear_right_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();  //GetWorldPose().Ign().Pos();
    auto front_left_center_pos = front_left_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();  //GetWorldPose().Ign().Pos();
    auto front_center_center_pos = front_center_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();  //GetWorldPose().Ign().Pos();
    auto front_right_center_pos = front_right_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();  //GetWorldPose().Ign().Pos();

    // track widths are computed first
    auto vec3 = front_left_center_pos - front_right_center_pos;
    front_track_width_ = vec3.Length();
    vec3 = rear_left_center_pos - rear_right_center_pos;
    rear_track_width_ = vec3.Length();

    // to compute wheelbase, first position of axle centers are computed
    auto front_axle_pos = (front_left_center_pos + front_right_center_pos) / 2;
    auto back_axle_pos = (rear_left_center_pos + rear_right_center_pos) / 2;
    // then the wheelbase is the distance between the axle centers
    vec3 = front_axle_pos - back_axle_pos;
    wheel_base_length_ = vec3.Length();

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&RoadrollerControlPlugin::Update, this));
  }

  void RoadrollerControlPlugin::controlCallback(const ackermann_msgs::AckermannDrive &msg)
  {
    //TODO program control logic -> when it is neccesary to apply brake force and when it is only necessary to reduce velocity, among other things

    std::lock_guard<std::mutex> lock{mutex_};
    ackermann_cmd_ = msg;
    if (ackermann_cmd_.speed > 0 && ackermann_cmd_.speed <= 0.01)
    {
      ackermann_cmd_.speed = 0; //This captures the dead pedal band on the real vehicle.
    }
  }

  void RoadrollerControlPlugin::Update()
  {
    std::lock_guard<std::mutex> lock{mutex_};

    auto cur_time = world_->SimTime(); //GetSimTime();
    auto dt = (cur_time - last_sim_time_).Double();
    if (dt < 0)
    {
      // TODO: reset
      return;
    }
    else if (dt == 0.0)
    {
      // TODO: use ignition::math::equal?
      return;
    }

    auto front_left_steering_angle = front_left_wheel_steering_joint_->Position(0); //GetAngle(0).Radian();
    auto front_center_steering_angle = front_center_wheel_steering_joint_->Position(0);
    auto front_right_steering_angle = front_right_wheel_steering_joint_->Position(0);

    auto rear_left_wheel_angular_velocity = rear_left_wheel_joint_->GetVelocity(0);
    auto rear_inner_left_wheel_angular_velocity = rear_inner_left_wheel_joint_->GetVelocity(0);
    auto rear_inner_right_wheel_angular_velocity = rear_inner_right_wheel_joint_->GetVelocity(0);
    auto rear_right_wheel_angular_velocity = rear_right_wheel_joint_->GetVelocity(0);
    auto front_left_wheel_angular_velocity = front_left_wheel_joint_->GetVelocity(0);
    auto front_center_wheel_angular_velocity = front_center_wheel_joint_->GetVelocity(0);
    auto front_right_wheel_angular_velocity = front_right_wheel_joint_->GetVelocity(0);

    // at this time we are going to live without drag force

    auto current_vel = chassis_link_->WorldCoGLinearVel(); //GetWorldCoGLinearVel();

    ROS_INFO("current velocity %f", current_vel.X());

    /* auto drag_force = -chassis_aero_force_gain_
      * chassis_linear_velocity.GetSquaredLength()
      * chassis_linear_velocity.Normalize();
    chassis_link_->AddForce(drag_force); */

    // auto steer_ratio = std::max(-100.0, std::min(100.0, (double)ackermann_cmd_.steering_angle)) / 100.0;
    // auto steer_angle = steer_ratio * max_steer_;
    auto steer_angle = ackermann_cmd_.steering_angle;

    // Ackermann steering geometry
    auto tan_steer = std::tan(steer_angle);
    auto front_left_wheel_steering_command =
        std::atan2(tan_steer, 1.0 + front_track_width_ / 2 / wheel_base_length_ * tan_steer);
    auto front_center_wheel_steering_command = steer_angle;
    auto front_right_wheel_steering_command =
        std::atan2(tan_steer, 1.0 - front_track_width_ / 2 / wheel_base_length_ * tan_steer);

    // Update steering PID controllers
    auto front_left_steering_error = front_left_steering_angle - front_left_wheel_steering_command;
    auto front_center_steering_error = front_center_steering_angle - front_center_wheel_steering_command;
    auto front_right_steering_error = front_right_steering_angle - front_right_wheel_steering_command;

    auto front_left_wheel_steering_force = front_left_wheel_steering_pid_.Update(front_left_steering_error, dt);
    auto front_center_wheel_steering_force = front_center_wheel_steering_pid_.Update(front_center_steering_error, dt);
    auto front_right_wheel_steering_force = front_right_wheel_steering_pid_.Update(front_right_steering_error, dt);

    front_left_wheel_steering_joint_->SetForce(0, front_left_wheel_steering_force);
    front_center_wheel_steering_joint_->SetForce(0, front_center_wheel_steering_force);
    front_right_wheel_steering_joint_->SetForce(0, front_right_wheel_steering_force);

    // speed and braking calculations

    // We have to obtain the angular velocity for each wheel

    double rear_left_wheel_ang_vel;
    double rear_inner_left_wheel_ang_vel;
    double rear_inner_right_wheel_ang_vel;
    double rear_right_wheel_ang_vel;
    //it is necessary to know the distance among the center of the rear axis and the instant center of rotation

    if (steer_angle != 0)
    {
      double distance_2_ICoR = wheel_base_length_ * std::tan(steer_angle);

      //the desired angular velocity of the vehicle is

      double vehicle_ang_vel = ackermann_cmd_.speed / distance_2_ICoR;

      // the desired velocity of the center of the rear axis is ackkermann_cmd.speed, but each wheel has its own linear velocity, depending of its distance to the ICoR
      // so, the linear velocity is calculated from the angular velocity of the vehicle multiplied with the distance of each wheel to the ICoR. Then,
      // this value is divided by the radius wheel to obtain the angular velocity of each wheel

      rear_left_wheel_ang_vel = ((distance_2_ICoR - rear_track_width_ / 2) * vehicle_ang_vel) / rear_left_wheel_radius_;
      rear_inner_left_wheel_ang_vel = ((distance_2_ICoR - rear_track_width_ / 4) * vehicle_ang_vel) / rear_inner_left_wheel_radius_;
      rear_inner_left_wheel_ang_vel = ((distance_2_ICoR + rear_track_width_ / 4) * vehicle_ang_vel) / rear_inner_right_wheel_radius_;
      rear_left_wheel_ang_vel = ((distance_2_ICoR + rear_track_width_ / 2) * vehicle_ang_vel) / rear_right_wheel_radius_;
    }
    else
    {
      rear_left_wheel_ang_vel = ackermann_cmd_.speed / rear_left_wheel_radius_;
      rear_inner_left_wheel_ang_vel = ackermann_cmd_.speed / rear_inner_left_wheel_radius_;
      rear_inner_left_wheel_ang_vel = ackermann_cmd_.speed / rear_inner_right_wheel_radius_;
      rear_left_wheel_ang_vel = ackermann_cmd_.speed / rear_right_wheel_radius_;
    }

    double throttle_ratio = 0.0;
    double brake_ratio = 0.0;
    double vel_error = 0.0;

    if (ackermann_cmd_.speed == 0)
    {
      // frenar
      brake_ratio = 1.0;
      ROS_INFO("braking. Park mode");
    }
    // if current vel and ackermann_cmd_.speed has the same direction, we check the absolute error in order to
    // decide if it is neccesary to apply braking force

    if (((current_vel.X() > 0) && (ackermann_cmd_.speed > 0)) ||
        (current_vel.X() < 0) && (ackermann_cmd_.speed < 0))
    {
      vel_error = std::abs(current_vel.X()) - std::abs(ackermann_cmd_.speed);
      if (vel_error > start_braking_threshold_)
      {
        if (vel_error > vel_error_max_braking_)
        {
          brake_ratio = 1.0;
          ROS_INFO("braking vel_error > vel_error_max_braking");
        }
        else
        {
          brake_ratio = (vel_error / vel_error_max_braking_);
        }
      }
    }

    if (ackermann_cmd_.speed > 0)
      throttle_ratio = std::min(100.0, (double)ackermann_cmd_.speed * 100.0) / 100.0;
    // if (ackermann_cmd_.speed < 0)
    //   brake_ratio = std::min(100.0, (double)-ackermann_cmd_.speed*100.0) / 100.0;

    // use regen braking, unless it is overcome by throttle, or direct brake ratio is higher
    // examples:
    // * throttle = 0, brake = 0 ==> brake_ratio = regen_braking_ratio
    // * throttle = 0, brake > regen ==> brake_ratio = brake_ratio
    // * throttle > regen_braking_ratio, brake_ratio = 0 ==> brake_ratio = 0
    /* brake_ratio = std::max(regen_braking_ratio - throttle_ratio, brake_ratio);
    brake_ratio = std::max(0.0, std::min(1.0, brake_ratio));*/

    rear_left_wheel_joint_->SetParam("friction", 0, brake_ratio * back_brake_torque_);
    rear_inner_left_wheel_joint_->SetParam("friction", 0, brake_ratio * back_brake_torque_);
    rear_inner_right_wheel_joint_->SetParam("friction", 0, brake_ratio * back_brake_torque_);
    rear_right_wheel_joint_->SetParam("friction", 0, brake_ratio * back_brake_torque_);
    front_left_wheel_joint_->SetParam("friction", 0, brake_ratio * front_brake_torque_);
    front_center_wheel_joint_->SetParam("friction", 0, brake_ratio * front_brake_torque_);
    front_right_wheel_joint_->SetParam("friction", 0, brake_ratio * front_brake_torque_);

    auto throttle_torque = 0.0;
    if (std::abs(rear_left_wheel_angular_velocity * rear_left_wheel_radius_) < max_speed_ &&
        std::abs(rear_right_wheel_angular_velocity * rear_right_wheel_radius_) < max_speed_)
      throttle_torque = throttle_ratio * max_torque_;

    rear_left_wheel_joint_->SetForce(0, throttle_torque);
    rear_inner_left_wheel_joint_->SetForce(0, throttle_torque);
    rear_inner_right_wheel_joint_->SetForce(0, throttle_torque);
    rear_right_wheel_joint_->SetForce(0, throttle_torque);

    last_sim_time_ = cur_time;
  }

  double RoadrollerControlPlugin::collisionRadius(gazebo::physics::CollisionPtr coll) const
  {
    if (coll == nullptr || coll->GetShape() == nullptr)
      return 0;

    if (coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE))
    {
      gazebo::physics::CylinderShape *cyl =
          static_cast<gazebo::physics::CylinderShape *>(coll->GetShape().get());
      return cyl->GetRadius();
    }
    else if (coll->GetShape()->HasType(gazebo::physics::Base::SPHERE_SHAPE))
    {
      gazebo::physics::SphereShape *sph =
          static_cast<gazebo::physics::SphereShape *>(coll->GetShape().get());
      return sph->GetRadius();
    }
    return 0;
  }

  GZ_REGISTER_MODEL_PLUGIN(RoadrollerControlPlugin)

} // namespace ROADROLLER_CONTROL