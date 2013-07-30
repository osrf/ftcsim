/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef _FTC2_HH_
#define _FTC2_HH_

#include <gazebo.hh>
#include <gazebo/physics/Physics.hh>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace gazebo
{
  class FTC2: public ModelPlugin
  {
    /// \brief Constructor
    public: FTC2();

    /// \brief Destructor
    public: ~FTC2();

    /// \brief
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Main loop
    public: void UpdateCB();

    /// \breif Receives jointsick messages.
    private: void JoystickCB(const sensor_msgs::Joy::ConstPtr &_msg);

    /// \brief Pointer to the DJI550 model.
    private: physics::ModelPtr model;

    /// \brief Pointers to the different camera stabilizer joints.
    private: physics::LinkPtr frame;

    /// \brief The grippper lift joint.
    private: physics::JointPtr liftJoint;

    /// \brief The gripper joint.
    private: physics::JointPtr clawJoint;

    /// \brief Pointer to the world update event.
    private: event::ConnectionPtr updateConn;

    /// \brief Pointer to the ROS node.
    private: ros::NodeHandle *rosNode;

    /// \brief Reads joystick data.
    private: ros::Subscriber joystickSub;

    /// \brief Forward values.
    private: double fwd;

    /// \brief Side value.
    private: double side;

    /// \brief Rotate value.
    private: double rotate;

    /// \brief Lift value.
    private: double lift;

    /// \brief Claw value.
    private: double claw;

    /// \brief Lift height.
    private: double height;
  };
}
#endif
