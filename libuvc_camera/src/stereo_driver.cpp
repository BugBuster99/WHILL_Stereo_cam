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
#include "libuvc_camera/stereo_driver.h"
#include "xunit/xunit_lib.h"

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <image_transport/camera_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <libuvc/libuvc.h>
#define libuvc_VERSION (libuvc_VERSION_MAJOR * 10000 \
                      + libuvc_VERSION_MINOR * 100 \
                      + libuvc_VERSION_PATCH)

namespace libuvc_camera {

StereoDriver::StereoDriver(ros::NodeHandle nh, ros::NodeHandle priv_nh)
  : nh_(nh), priv_nh_(priv_nh),
    state_(kInitial),
    ctx_(NULL), dev_(NULL), devh_(NULL), rgb_frame_(NULL), ctrl_(), hid_fd_(0), hid_device_(),
    it_(nh_),
    config_server_(mutex_, priv_nh_),
    config_changed_(false),
    cinfo_manager_(nh) {
  cam_pub_ = it_.advertiseCamera("image_raw", 1, false);
  set_stream_srv_ = nh_.advertiseService("set_stream_mode",&StereoDriver::SetStreamMode,this);
  get_stream_srv_ = nh_.advertiseService("get_stream_mode",&StereoDriver::GetStreamMode,this);
  get_fwver_srv_  = nh_.advertiseService("get_firmware_version", &StereoDriver::GetFirmwareVersion, this);
  set_frame_rate_srv_ = nh_.advertiseService("set_frame_rate",&StereoDriver::SetFrameRate,this);
  get_frame_rate_srv_ = nh_.advertiseService("get_frame_rate",&StereoDriver::GetFrameRate,this);
}

StereoDriver::~StereoDriver() {
  if (rgb_frame_)
    uvc_free_frame(rgb_frame_);

  if (ctx_)
    uvc_exit(ctx_);  // Destroys dev_, devh_, etc.
}

bool StereoDriver::Start() {
  assert(state_ == kInitial);

  uvc_error_t err;

  err = uvc_init(&ctx_, NULL);

  if (err != UVC_SUCCESS) {
    uvc_perror(err, "ERROR: uvc_init");
    return false;
  }

  state_ = kStopped;

  config_server_.setCallback(boost::bind(&StereoDriver::ReconfigureCallback, this, _1, _2));

  return state_ == kRunning;
}

void StereoDriver::Stop() {
  boost::recursive_mutex::scoped_lock(mutex_);

  assert(state_ != kInitial);

  if (state_ == kRunning)
    CloseCamera();

  assert(state_ == kStopped);

  uvc_exit(ctx_);
  ctx_ = NULL;

  state_ = kInitial;
}

void StereoDriver::ReconfigureCallback(UVCCameraConfig &new_config, uint32_t level) {
  boost::recursive_mutex::scoped_lock(mutex_);

  if ((level & kReconfigureClose) == kReconfigureClose) {
    if (state_ == kRunning)
      CloseCamera();
  }

  if (state_ == kStopped) {
    OpenCamera(new_config);
  }

  if (new_config.camera_info_url != config_.camera_info_url)
    cinfo_manager_.loadCameraInfo(new_config.camera_info_url);

  if (state_ == kRunning) {
#define PARAM_INT(name, fn, value) if (new_config.name != config_.name) { \
      int val = (value);                                                \
      if (uvc_set_##fn(devh_, val)) {                                   \
        ROS_WARN("Unable to set " #name " to %d", val);                 \
        new_config.name = config_.name;                                 \
      }                                                                 \
    }

    PARAM_INT(scanning_mode, scanning_mode, new_config.scanning_mode);
    PARAM_INT(auto_exposure, ae_mode, 1 << new_config.auto_exposure);
    PARAM_INT(auto_exposure_priority, ae_priority, new_config.auto_exposure_priority);
    PARAM_INT(exposure_absolute, exposure_abs, new_config.exposure_absolute * 10000);
    PARAM_INT(auto_focus, focus_auto, new_config.auto_focus ? 1 : 0);
    PARAM_INT(focus_absolute, focus_abs, new_config.focus_absolute);
#if libuvc_VERSION     > 00005 /* version > 0.0.5 */
    PARAM_INT(gain, gain, new_config.gain);
    PARAM_INT(iris_absolute, iris_abs, new_config.iris_absolute);
    PARAM_INT(brightness, brightness, new_config.brightness);
#endif
    

    if (new_config.pan_absolute != config_.pan_absolute || new_config.tilt_absolute != config_.tilt_absolute) {
      if (uvc_set_pantilt_abs(devh_, new_config.pan_absolute, new_config.tilt_absolute)) {
        ROS_WARN("Unable to set pantilt to %d, %d", new_config.pan_absolute, new_config.tilt_absolute);
        new_config.pan_absolute = config_.pan_absolute;
        new_config.tilt_absolute = config_.tilt_absolute;
      }
    }
    // TODO: roll_absolute
    // TODO: privacy
    // TODO: backlight_compensation
    // TODO: contrast
    // TODO: power_line_frequency
    // TODO: auto_hue
    // TODO: saturation
    // TODO: sharpness
    // TODO: gamma
    // TODO: auto_white_balance
    // TODO: white_balance_temperature
    // TODO: white_balance_BU
    // TODO: white_balance_RV
  }

  config_ = new_config;
}

void StereoDriver::ImageCallback(uvc_frame_t *frame) {
  ros::Time timestamp = ros::Time(frame->capture_time.tv_sec, frame->capture_time.tv_usec);
  if ( timestamp == ros::Time(0) ) {
    timestamp = ros::Time::now();
  }

  boost::recursive_mutex::scoped_lock(mutex_);

  assert(state_ == kRunning);
  assert(rgb_frame_);

  sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
  image->width = config_.width;
  image->height = config_.height;
  if(frame->frame_format == UVC_FRAME_FORMAT_GRAY8)
  {
    image->step = image->width;
  }
  else
  {
    image->step = image->width * 3;
  }
  image->data.resize(image->step * image->height);

  if (frame->frame_format == UVC_FRAME_FORMAT_BGR){
    image->encoding = "bgr8";
    memcpy(&(image->data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_RGB){
    image->encoding = "rgb8";
    memcpy(&(image->data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_UYVY) {
    image->encoding = "yuv422";
    memcpy(&(image->data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_YUYV) {
    // FIXME: uvc_any2bgr does not work on "yuyv" format, so use uvc_yuyv2bgr directly.
    uvc_error_t conv_ret = uvc_yuyv2bgr(frame, rgb_frame_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    image->encoding = "bgr8";
    memcpy(&(image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);
#if libuvc_VERSION     > 00005 /* version > 0.0.5 */
  } else if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
    // Enable mjpeg support despite uvs_any2bgr shortcoming
    //  https://github.com/ros-drivers/libuvc_ros/commit/7508a09f
    uvc_error_t conv_ret = uvc_mjpeg2rgb(frame, rgb_frame_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    image->encoding = "rgb8";
    memcpy(&(image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);
#endif
  } else if (frame->frame_format == UVC_FRAME_FORMAT_GRAY8) {
    image->encoding = "mono8";
    memcpy(&(image->data[0]), frame->data, image->width * image->height);
  } else {
    uvc_error_t conv_ret = uvc_any2bgr(frame, rgb_frame_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    image->encoding = "bgr8";
    memcpy(&(image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);
  }


  sensor_msgs::CameraInfo::Ptr cinfo(
    new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));

  image->header.frame_id = config_.frame_id;
  image->header.stamp = timestamp;
  cinfo->header.frame_id = config_.frame_id;
  cinfo->header.stamp = timestamp;

  cam_pub_.publish(image, cinfo);

  if (config_changed_) {
    config_server_.updateConfig(config_);
    config_changed_ = false;
  }
}

/* static */ void StereoDriver::ImageCallbackAdapter(uvc_frame_t *frame, void *ptr) {
  StereoDriver *driver = static_cast<StereoDriver*>(ptr);

  driver->ImageCallback(frame);
}

void StereoDriver::AutoControlsCallback(
  enum uvc_status_class status_class,
  int event,
  int selector,
  enum uvc_status_attribute status_attribute,
  void *data, size_t data_len) {
  boost::recursive_mutex::scoped_lock(mutex_);

  printf("Controls callback. class: %d, event: %d, selector: %d, attr: %d, data_len: %zu\n",
         status_class, event, selector, status_attribute, data_len);

  if (status_attribute == UVC_STATUS_ATTRIBUTE_VALUE_CHANGE) {
    switch (status_class) {
    case UVC_STATUS_CLASS_CONTROL_CAMERA: {
      switch (selector) {
      case UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL:
        uint8_t *data_char = (uint8_t*) data;
        uint32_t exposure_int = ((data_char[0]) | (data_char[1] << 8) |
                                 (data_char[2] << 16) | (data_char[3] << 24));
        config_.exposure_absolute = exposure_int * 0.0001;
        config_changed_ = true;
        break;
      }
      break;
    }
    case UVC_STATUS_CLASS_CONTROL_PROCESSING: {
      switch (selector) {
      case UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
        uint8_t *data_char = (uint8_t*) data;
        config_.white_balance_temperature = 
          data_char[0] | (data_char[1] << 8);
        config_changed_ = true;
        break;
      }
      break;
    }
    }

    // config_server_.updateConfig(config_);
  }
}

/* static */ void StereoDriver::AutoControlsCallbackAdapter(
  enum uvc_status_class status_class,
  int event,
  int selector,
  enum uvc_status_attribute status_attribute,
  void *data, size_t data_len,
  void *ptr) {
  StereoDriver *driver = static_cast<StereoDriver*>(ptr);

  driver->AutoControlsCallback(status_class, event, selector,
                               status_attribute, data, data_len);
}

enum uvc_frame_format StereoDriver::GetVideoMode(std::string vmode){
  if(vmode == "uncompressed") {
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  } else if (vmode == "compressed") {
    return UVC_COLOR_FORMAT_COMPRESSED;
  } else if (vmode == "yuyv") {
    return UVC_COLOR_FORMAT_YUYV;
  } else if (vmode == "uyvy") {
    return UVC_COLOR_FORMAT_UYVY;
  } else if (vmode == "rgb") {
    return UVC_COLOR_FORMAT_RGB;
  } else if (vmode == "bgr") {
    return UVC_COLOR_FORMAT_BGR;
  } else if (vmode == "mjpeg") {
    return UVC_COLOR_FORMAT_MJPEG;
  } else if (vmode == "gray8") {
    return UVC_COLOR_FORMAT_GRAY8;
  } else {
    ROS_ERROR_STREAM("Invalid Video Mode: " << vmode);
    ROS_WARN_STREAM("Continue using video mode: uncompressed");
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  }
};

void StereoDriver::OpenCamera(UVCCameraConfig &new_config) {
  assert(state_ == kStopped);

  int vendor_id = strtol(new_config.vendor.c_str(), NULL, 0);
  int product_id = strtol(new_config.product.c_str(), NULL, 0);

  ROS_INFO("Opening camera with vendor=0x%x, product=0x%x, serial=\"%s\", index=%d",
           vendor_id, product_id, new_config.serial.c_str(), new_config.index);

  uvc_device_t **devs;

  // Implement missing index select behavior
  // https://github.com/ros-drivers/libuvc_ros/commit/4f30e9a0
#if libuvc_VERSION     > 00005 /* version > 0.0.5 */
  uvc_error_t find_err = uvc_find_devices(
    ctx_, &devs,
    vendor_id,
    product_id,
    new_config.serial.empty() ? NULL : new_config.serial.c_str());

  if (find_err != UVC_SUCCESS) {
    uvc_perror(find_err, "uvc_find_device");
    return;
  }

  // select device by index
  dev_ = NULL;
  int dev_idx = 0;
  while (devs[dev_idx] != NULL) {
    if(dev_idx == new_config.index) {
      dev_ = devs[dev_idx];
    }
    else {
      uvc_unref_device(devs[dev_idx]);
    }

    dev_idx++;
  }

  if(dev_ == NULL) {
    ROS_ERROR("Unable to find device at index %d", new_config.index);
    return;
  }
#else
  uvc_error_t find_err = uvc_find_device(
    ctx_, &dev_,
    vendor_id,
    product_id,
    new_config.serial.empty() ? NULL : new_config.serial.c_str());

  if (find_err != UVC_SUCCESS) {
    uvc_perror(find_err, "uvc_find_device");
    return;
  }

#endif
  uvc_error_t open_err = uvc_open(dev_, &devh_);

  if (open_err != UVC_SUCCESS) {
    switch (open_err) {
    case UVC_ERROR_ACCESS:
#ifdef __linux__
      ROS_ERROR("Permission denied opening /dev/bus/usb/%03d/%03d",
                uvc_get_bus_number(dev_), uvc_get_device_address(dev_));
#else
      ROS_ERROR("Permission denied opening device %d on bus %d",
                uvc_get_device_address(dev_), uvc_get_bus_number(dev_));
#endif
      break;
    default:
#ifdef __linux__
      ROS_ERROR("Can't open /dev/bus/usb/%03d/%03d: %s (%d)",
                uvc_get_bus_number(dev_), uvc_get_device_address(dev_),
                uvc_strerror(open_err), open_err);
#else
      ROS_ERROR("Can't open device %d on bus %d: %s (%d)",
                uvc_get_device_address(dev_), uvc_get_bus_number(dev_),
                uvc_strerror(open_err), open_err);
#endif
      break;
    }

    uvc_unref_device(dev_);
    return;
  }

  /* Open Extension Unit */
  hid_fd_ = InitExtensionUnit(new_config.serial.c_str());
  hid_device_ = std::string(hid_device);
  if(hid_fd_ == NULL_HANDLE)
  {
    ROS_ERROR("Extention Unit Failed to open");
  }
  else
  {
    g_FWver_t fwversion = {0};
    int ret = ReadFirmwareVersion (&hid_fd_, &fwversion);
    ROS_INFO("Firmware Version: %d.%d.%d.%d", fwversion.pMajorVersion, fwversion.pMinorVersion1, fwversion.pMinorVersion2, fwversion.pMinorVersion3);
    ret = SetFrameRateValue(&hid_fd_, static_cast<uint32_t>(new_config.frame_rate));
    ROS_INFO("Set Frame Rate: %f, result: %d", new_config.frame_rate, ret);
  }

  uvc_set_status_callback(devh_, &StereoDriver::AutoControlsCallbackAdapter, this);

  uvc_stream_ctrl_t ctrl;
  uvc_error_t mode_err = uvc_get_stream_ctrl_format_size(
    devh_, &ctrl,
    GetVideoMode(new_config.video_mode),
    new_config.width, new_config.height,
    new_config.frame_rate);

  if (mode_err != UVC_SUCCESS) {
    uvc_perror(mode_err, "uvc_get_stream_ctrl_format_size");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    ROS_ERROR("check video_mode/width/height/frame_rate are available");
    uvc_print_diag(devh_, NULL);
    return;
  }

  ctrl_ = ctrl;
  uvc_error_t stream_err = uvc_start_streaming(devh_, &ctrl_, &StereoDriver::ImageCallbackAdapter, this, 0);

  if (stream_err != UVC_SUCCESS) {
    uvc_perror(stream_err, "uvc_start_streaming");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    return;
  }

  if (rgb_frame_)
    uvc_free_frame(rgb_frame_);

  rgb_frame_ = uvc_allocate_frame(new_config.width * new_config.height * 3);
  assert(rgb_frame_);

  state_ = kRunning;
}

void StereoDriver::CloseCamera() {
  assert(state_ == kRunning);

  uvc_close(devh_);
  devh_ = NULL;

  uvc_unref_device(dev_);
  dev_ = NULL;

  state_ = kStopped;
}

bool StereoDriver::SetStreamMode(libuvc_camera::SetStreamMode::Request &req,
                                 libuvc_camera::SetStreamMode::Response &res)
{
  ROS_INFO("%s: HID Handle=%d, HID Device=%s", __func__, hid_fd_, hid_device_.c_str());
  ROS_INFO("Command String: %s", req.mode_command.c_str());
  
  int result = 0;
  bool ret = true;
  std::string command = req.mode_command; 
  std::transform(command.begin(), command.end(), command.begin(), ::toupper);

  if(std::equal(command.begin(), command.end(), "MASTER") == true)
  {
    result = Master_Mode(&hid_fd_);
  }
  else if(std::equal(command.begin(), command.end(), "TRIGGER") == true)
  {
    result = Trigger_Mode(&hid_fd_);
  }
  else
  {
    ROS_WARN("Invalid Command. Acceptable command is 'Master' or 'Trigger' (Case insensitive).");
    ret = false;
  }
  res.result = result;
  return ret;
} 

bool StereoDriver::GetFirmwareVersion(libuvc_camera::GetFirmwareVersion::Request &req,
                                      libuvc_camera::GetFirmwareVersion::Response &res)
{
  ROS_INFO("%s: HID Handle=%d, HID Device=%s", __func__, hid_fd_, hid_device_.c_str());
  g_FWver_t fwversion = {0};
  int ret = ReadFirmwareVersion (&hid_fd_, &fwversion);
  ROS_INFO("Firmware Version: %d.%d.%d.%d", fwversion.pMajorVersion, fwversion.pMinorVersion1, fwversion.pMinorVersion2, fwversion.pMinorVersion3);
  if(ret == FAIL)
  {
    return false;
  }
  else
  {
    res.major_version   = fwversion.pMajorVersion;
    res.minor_version_1 = fwversion.pMinorVersion1;
    res.minor_version_2 = fwversion.pMinorVersion2;
    res.minor_version_3 = fwversion.pMinorVersion3;
    return true;
  }
}

bool StereoDriver::GetStreamMode(libuvc_camera::GetStreamMode::Request &req,
                                 libuvc_camera::GetStreamMode::Response &res)
{
  ROS_INFO("%s: HID Handle=%d, HID Device=%s", __func__, hid_fd_, hid_device_.c_str());
  int exposure;
  int mode;
  int ret = Get_Trigger_Mode(&hid_fd_, &mode, &exposure);
  if(ret == FAIL)
  {
    return false;
  }
  else
  {
    res.stream_mode = mode;
    res.exposure = exposure;
    return true;
  }
}

bool StereoDriver::SetFrameRate(libuvc_camera::SetFrameRate::Request &req,
                                libuvc_camera::SetFrameRate::Response &res)
{
  ROS_INFO("%s: HID Handle=%d, HID Device=%s", __func__, hid_fd_, hid_device_.c_str());
  ROS_INFO("Frame Rate Requested: %u", req.frame_rate);
  
  res.result = SetFrameRateValue(&hid_fd_, req.frame_rate);
  if(res.result == PASS)
  {
    ROS_INFO("Frame Rate successfully set.");
  }
  else if(res.result == OUT_OF_RANGE)
  {
    ROS_WARN("Requested frame rate is out of range");
  }
  else
  {
    ROS_ERROR("%s failed", __func__);
  }
  return true;
} 

bool StereoDriver::GetFrameRate(libuvc_camera::GetFrameRate::Request &req,
                                libuvc_camera::GetFrameRate::Response &res)
{
  ROS_INFO("%s: HID Handle=%d, HID Device=%s", __func__, hid_fd_, hid_device_.c_str());
  
  uint32_t frame_rate;
  res.result = GetFrameRateValue(&hid_fd_, &frame_rate);
  if(res.result == PASS)
  {
    res.frame_rate = frame_rate;
    ROS_INFO("Frame Rate Returned: %u", res.frame_rate);
    return true;
  }
  else
  {
    ROS_ERROR("%s failed", __func__);
    return false;
  }
} 





};
