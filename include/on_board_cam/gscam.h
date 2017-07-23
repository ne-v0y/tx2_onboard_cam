#ifndef __GSCAM_GSCAM_H
#define __GSCAM_GSCAM_H

extern "C"{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include "gstCamera.h"
#include "cudaNormalize.h"

// STD include
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdexcept>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <iostream>
#include "utils.h"

// ROS include
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <camera_calibration_parsers/parse_ini.h>

// OpenCV include
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

namespace au_core {

class GSCam {
  
public:
    GSCam(ros::NodeHandle nh_camera, ros::NodeHandle nh_private);
    ~GSCam();

    gstCamera* myCam_ = NULL;

    void* imgCPU  = NULL;
    void* imgCUDA = NULL;
    void* imgRGBA = NULL;

    bool configure();
    void init_stream();
    void publish_stream();

    void run();
    void close();


private:
    // General gstreamer configuration
    std::string gsconfig_;

    // Camera publisher configuration
    cv::Mat frame_;
    cv::Mat frame_out_;
    std::string frame_id_;
    int width_, height_;
    std::string image_encoding_;
    std::string camera_name_;
    std::string camera_info_name_;
    std::string camera_info_url_;

    // ROS Inteface
    // Calibration between ros::Time and gst timestamps
    double time_offset_;
    ros::NodeHandle nh_, nh_private_;
    image_transport::ImageTransport image_transport_;
    camera_info_manager::CameraInfoManager camera_info_manager_;
    //image_transport::CameraPublisher camera_pub_;
    
    // Image and Info publisher
    ros::Publisher bottom_pub_;
    ros::Publisher cinfo_pub_;
  };

} // end of namesapce

#endif // ifndef __GSCAM_GSCAM_H
