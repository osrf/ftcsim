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

#ifndef _URG04LX_H_
#define _URG04LX_H_

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/Sensors.hh>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace gazebo
{
  class CameraPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: CameraPlugin();

    /// \brief Destructor
    public: ~CameraPlugin();

    /// \brief Performs load time initialization.
    /// \param[in] _sensor Pointer to the parent sensor.
    /// \param[in] _sdf SDF parameters.
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Plugin update callback.
    private: void UpdateCB();

    /// \brief The simulated camera.
    private: sensors::CameraSensorPtr camera;

    /// \brief Simulation update connection.
    private: event::ConnectionPtr updateConn;

    /// \brief Pointer to the ROS node.
    private: ros::NodeHandle* rosNode;

    /// \brief Image transport.
    private: image_transport::ImageTransport *imageTransport;

    /// \brief Image publisher.
    private: image_transport::Publisher imagePub;
  };
}
#endif
