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

#include <gazebo/physics/Physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/Camera.hh>
#include <sensor_msgs/image_encodings.h>

#include "CameraPlugin.hh"

using namespace gazebo;

// Register the plugin with the simulator.
GZ_REGISTER_SENSOR_PLUGIN(CameraPlugin)

  /////////////////////////////////////////////////
CameraPlugin::CameraPlugin()
{
}

/////////////////////////////////////////////////
CameraPlugin::~CameraPlugin()
{
  delete this->rosNode;
  delete this->imageTransport;
}

/////////////////////////////////////////////////
void CameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Store pointer to sensor.
  this->camera = boost::shared_dynamic_cast<sensors::CameraSensor>(_sensor);

  // Start up ROS.
  int argc = 0;
  const std::string NodeName = "ROSGazeboInterface";
  ros::init(argc, NULL, NodeName);

  // Create the ROS Node in the system.
  std::string modelName = physics::get_world(
      this->camera->GetWorldName())->GetEntity(
      this->camera->GetParentName())->GetParentModel()->GetName();

  std::stringstream ss;
  ss << modelName << "/" << this->camera->GetName();

  this->rosNode = new ros::NodeHandle(ss.str());

  // Initialize the internal ROS publishers
  this->imageTransport = new image_transport::ImageTransport(*this->rosNode);

  this->imagePub = this->imageTransport->advertise("Video", 1);

  // Connect to the world update event.
  this->updateConn = this->camera->ConnectUpdated(
      boost::bind(&CameraPlugin::UpdateCB, this));
}

/////////////////////////////////////////////////
void CameraPlugin::UpdateCB()
{
  // Build up Image msg.
  sensor_msgs::Image msg;

  std_msgs::Header hdr;
  hdr.stamp = ros::Time::now();
  hdr.frame_id = "map";
  msg.header = hdr;

  msg.height = this->camera->GetImageHeight();
  msg.width = this->camera->GetImageWidth();

  msg.encoding = "rgb8";

  unsigned int imageSize = this->camera->GetCamera()->GetImageByteSize();
  const unsigned char* imageData = this->camera->GetImageData();

  msg.step = imageSize/msg.height;
  msg.data.assign(imageData, imageData + imageSize);

  this->imagePub.publish(msg);
}
