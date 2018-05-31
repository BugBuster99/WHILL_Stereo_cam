/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2012 Ken Tossell
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "libuvc_camera/stereo_driver.h"

namespace libuvc_camera {

class StereoNodelet : public nodelet::Nodelet {
public:
  StereoNodelet() : running_(false) {}
  ~StereoNodelet();

private:
  virtual void onInit();

  volatile bool running_;
  boost::shared_ptr<StereoDriver> driver_;
};

StereoNodelet::~StereoNodelet() {
  if (running_) {
    driver_->Stop();
  }
}

void StereoNodelet::onInit() {
  ros::NodeHandle nh(getNodeHandle());
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  driver_.reset(new StereoDriver(nh, priv_nh));
  if (driver_->Start()) {
    running_ = true;
  } else {
    NODELET_ERROR("Unable to open camera.");
    driver_.reset();
  }
}

};

// Register this plugin with pluginlib.
//
// parameters are: class type, base class type
PLUGINLIB_EXPORT_CLASS(libuvc_camera::StereoNodelet, nodelet::Nodelet)