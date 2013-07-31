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
 * See the License for the specific language governing permissions and * limitations under the License.
 *
 */

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/PhysicsEngine.hh>

#include <sensor_msgs/Joy.h>

#include "FTC.hh"

using namespace gazebo;

// Register the plugin with the simulator.
GZ_REGISTER_MODEL_PLUGIN(FTC)

/////////////////////////////////////////////////
FTC::FTC()
{
  this->fwd = 0;
  this->side = 0;
  this->rotate = 0;
  this->lift = 0;
  this->claw = 0;
  this->height = 0.5;
}

/////////////////////////////////////////////////
FTC::~FTC()
{
  // delete the ROS Node pointer.
  delete this->rosNode;
}

/////////////////////////////////////////////////
void FTC::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Store the model pointer.
  this->model = _model;
  this->frame = this->model->GetLink("chassis");

  // Get the joint pointers.
  this->liftJoint = this->model->GetJoint("lift_joint");
  this->clawJoint = this->model->GetJoint("claw_joint");

  // Start up ROS.
  int argc = 0;
  const std::string NodeName = "ROSGazeboInterface";
  ros::init(argc, NULL, NodeName);

  // Create the ROS Node in the system.
  this->rosNode = new ros::NodeHandle(this->model->GetScopedName());

  std::string joyTopic = "/ftc_robot/joy";
  if (_sdf->HasElement("joy_topic"))
    joyTopic = _sdf->Get<std::string>("joy_topic");

  // Initialize the internal ROS subscribers.
  this->joystickSub = this->rosNode->subscribe<sensor_msgs::Joy>(joyTopic,
      128, &FTC::JoystickCB, this);

  // Connect to the world update event.
  this->updateConn = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&FTC::UpdateCB, this));
}

/////////////////////////////////////////////////
void FTC::JoystickCB(const sensor_msgs::Joy::ConstPtr &_msg)
{
  this->fwd = _msg->axes[1] * 150.0;
  this->side = _msg->axes[0] * -150.0;
  this->rotate = _msg->axes[2] * 5.0;
  this->lift = 0;
  this->claw = 0;
  this->height = 0.5;

  if (_msg->buttons[2] == true)
    this->claw = _msg->axes[3] * 0.5;
  else
    this->lift = _msg->axes[3] * 10;

  if (_msg->axes[5] == -1)
    this->height = 0.5 - 0.3556;
  else if (_msg->axes[5] == 1)
    this->height = 0.5 + 0.3556;
  else
    this->height = 0.5;
}

/////////////////////////////////////////////////
void FTC::UpdateCB()
{
  // Perform ROS iteration.
  ros::spinOnce();

  // Apply forces and torques.
  this->frame->AddRelativeForce(math::Vector3(
        this->side * sin(-1.57/2.0) + this->fwd * cos(-1.57/2.0), 0,
        this->side * cos(-1.57/2.0) - this->fwd * sin(-1.57/2.0)));

  this->frame->AddRelativeTorque(math::Vector3(0, this->rotate, 0));

  this->liftJoint->SetForce(0, this->lift);
  this->clawJoint->SetForce(0, this->claw);
  this->liftJoint->SetHighStop(0, this->height);
}
