/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief NRF Plugin
 *
 * This plugin simulates nrf data
 *
 * @author Azeem Muzammil
 */

#include <gazebo_nrf_module_plugin.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(NRFPlugin)

NRFPlugin::NRFPlugin() : ModelPlugin(),
    nrf_rnd_y2_(0.0),
    nrf_rnd_use_last_(false)
{
}

NRFPlugin::~NRFPlugin()
{
  update_connection_->~Connection();
}

void NRFPlugin::getSdfParams(sdf::ElementPtr sdf)
{

  namespace_.clear();
  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_nrf_module_plugin] Please specify a robotNamespace.\n";
  }

  if (sdf->HasElement("pubRate")) {
    pub_rate_ = sdf->GetElement("pubRate")->Get<unsigned int>();
  } else {
    pub_rate_ = kDefaultPubRate;
    gzwarn << "[gazebo_nrf_module_plugin] Using default publication rate of " << pub_rate_ << " Hz\n";
  }

  if (sdf->HasElement("nrfTopic")) {
    nrf_topic_ = sdf->GetElement("nrfTopic")->Get<std::string>();
  } else {
    nrf_topic_ = kDefaultNRFTopic;
    gzwarn << "[gazebo_nrf_module_plugin] Using default nrf_module topic " << nrf_topic_ << "\n";
  }
}

void NRFPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  getSdfParams(sdf);

  model_ = model;
  world_ = model_->GetWorld();
#if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  last_pub_time_ = world_->SimTime();
#else
  last_time_ = world_->GetSimTime();
  last_pub_time_ = world_->GetSimTime();
#endif

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&NRFPlugin::OnUpdate, this, _1));

  pub_nrf_ = node_handle_->Advertise<sensor_msgs::msgs::NRF>("~/" + model_->GetName() + nrf_topic_, 10);

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 0.1);
}

void NRFPlugin::OnUpdate(const common::UpdateInfo&)
{
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif
  double dt = (current_time - last_pub_time_).Double();

  if (dt > 1.0 / pub_rate_) {

    //clear the message buffer
    nrf_msg_.clear_data();

    std::string message;

    for (int i = 0; i < 10; i++) {
      int randomNumber = rand() % 2;
      double noise = standard_normal_distribution_(random_generator_);
      float bitValue = randomNumber - noise;

      //set data
      nrf_msg_.add_data(bitValue);

      //set the message to log
      if (i == 0) {
        message = "[" + std::to_string(bitValue) + ", ";
      }
      else if (i == 9) {
        message = message + std::to_string(bitValue) + "]";
      }
      else {
        message = message + std::to_string(bitValue) + ", ";
      }
    }

    // Fill message
    nrf_msg_.set_time_usec(current_time.Double() * 1e6);

    last_pub_time_ = current_time;

    // Publish the message
    pub_nrf_->Publish(nrf_msg_);

    std:: cout << "The published message is: " << message << std::endl;
  }
}
}
