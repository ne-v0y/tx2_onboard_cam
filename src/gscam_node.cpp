// gscam node handler 
// created on: July 16 2017
// author: Noni Hua
// reference: ROS gscam_driver

#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <iostream>
extern "C"{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <camera_calibration_parsers/parse_ini.h>

#include <gscam/gscam.h>

namespace au_core {

  GSCam::GSCam(ros::NodeHandler nh_camera, ros::NodeHnadle nh_private) :
    nh_(nh_camera),
    nh_private_(nh_private),
    image_transport_(nh),
    camera_infor_manager_(nh)
  {
  }

  GSCam::~GSCam()
  {
  }
    



} // end of namespace
