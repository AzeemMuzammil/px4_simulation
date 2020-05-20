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
 * @brief BTest plugin
 *
 * This plugin simulates test data
 *
 * @author Azeem Muzammil
 */

#include <gazebo_b_test.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(BTestPlugin)

BTestPlugin::BTestPlugin() : ModelPlugin(),
    b_test_rnd_y2_(0.0),
    b_test_rnd_use_last_(false)
{
}

BTestPlugin::~BTestPlugin()
{
  update_connection_->~Connection();
}

void BTestPlugin::getSdfParams(sdf::ElementPtr sdf)
{

  namespace_.clear();
  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_b_test_plugin] Please specify a robotNamespace.\n";
  }

  if (sdf->HasElement("pubRate")) {
    pub_rate_ = sdf->GetElement("pubRate")->Get<unsigned int>();
  } else {
    pub_rate_ = kDefaultPubRate;
    gzwarn << "[gazebo_b_test_plugin] Using default publication rate of " << pub_rate_ << " Hz\n";
  }

  if (sdf->HasElement("bTestTopic")) {
    b_test_topic_ = sdf->GetElement("bTestTopic")->Get<std::string>();
  } else {
    b_test_topic_ = kDefaultBTestTopic;
    gzwarn << "[gazebo_b_test_plugin] Using default b_test topic " << b_test_topic_ << "\n";
  }
}

void BTestPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
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
      boost::bind(&BTestPlugin::OnUpdate, this, _1));

  pub_b_test_ = node_handle_->Advertise<sensor_msgs::msgs::BTest>("~/" + model_->GetName() + b_test_topic_, 10);

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
}

void BTestPlugin::OnUpdate(const common::UpdateInfo&)
{
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif
  double dt = (current_time - last_pub_time_).Double();

  if (dt > 1.0 / pub_rate_) {

    const float some_data = 288.0f;

    // generate Gaussian noise sequence using polar form of Box-Muller transformation
    double x1, x2, w, y1;
    if (!b_test_rnd_use_last_) {
      do {
        x1 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
        x2 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
        w = x1 * x1 + x2 * x2;
      } while ( w >= 1.0 );
      w = sqrt( (-2.0 * log( w ) ) / w );
      // calculate two values - the second value can be used next time because it is uncorrelated
      y1 = x1 * w;
      b_test_rnd_y2_ = x2 * w;
      b_test_rnd_use_last_ = true;
    } else {
      // no need to repeat the calculation - use the second value from last update
      y1 = b_test_rnd_y2_;
      b_test_rnd_use_last_ = false;
    }

    float data_local = some_data * (float)y1;

    //set data
    b_test_msg_.set_data(data_local);

    // Fill test msg
    b_test_msg_.set_time_usec(current_time.Double() * 1e6);

    last_pub_time_ = current_time;

    // Publish test msg
    pub_b_test_->Publish(b_test_msg_);
  }
}
}
