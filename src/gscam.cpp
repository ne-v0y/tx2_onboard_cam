// gscam node handler 
// created on: July 16 2017
// author: Noni Hua
// reference: ROS gscam_driver

#include <gscam.h>

using namespace cv;
using namespace std;

namespace au_core {

    GSCam::GSCam(ros::NodeHandle nh_camera, ros::NodeHandle nh_private) :
      nh_(nh_camera),
      nh_private_(nh_private)
      //image_transport_(nh_camera),
      //camera_info_manager_(nh_camera)
    {
        bottom_pub_ = nh_.advertise<sensor_msgs::Image>("/bottom/image/raw", 1);
        cinfo_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/bottom/camera_info",1);

        frame_ = Mat::zeros(height_, width_, CV_32FC3);

        init_stream();
    }

    GSCam::~GSCam()
    {
    }

    void GSCam::init_stream()
    {
        GSCam::myCam_ = gstCamera::Create();
        if( !myCam_ )
        {
            ROS_INFO("\ngst-camera: failed to initialize video device\n");
            exit(0);
        }

        height_ = helperlibs::Float2Int()(myCam_->GetHeight());
        width_ = helperlibs::Float2Int()(myCam_->GetWidth());   
	    
        if( !myCam_->Open() )
	    {
		    ROS_INFO("\ngst-camera:  failed to open camera for streaming\n");
		    exit(0);
	    }
	
	    ROS_INFO("\ngst-camera:  camera open for streaming\n");   
    }

    void GSCam::run()
    {
        // get the latest frame
		if( !myCam_->Capture(&imgCPU, &imgCUDA, 1000) )
			ROS_INFO("\ngst-camera:  failed to capture frame\n");
		//else
		//	ROS_INFO("gst-camera:  recieved new frame  CPU=0x%p  GPU=0x%p\n", imgCPU, imgCUDA);
	
		if( !myCam_->ConvertRGBA(imgCUDA, &imgRGBA, true) )
			ROS_INFO("gst-camera:  failed to convert from NV12 to RGBA\n"); 
        
        frame_ = Mat::zeros(height_, width_, CV_32FC3);
        helperlibs::Float42Mat(static_cast<float4*>(imgRGBA), frame_, width_, height_);

        frame_.convertTo(frame_out_, CV_8UC3);
        publish_stream();
    }

    void GSCam::publish_stream()
    {
        sensor_msgs::ImagePtr msg;
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_out_).toImageMsg();

        bottom_pub_.publish(msg); 
    }
    
    void GSCam::close()
    {
        ros::shutdown();
        if( myCam_ != NULL )
	    {
		    delete myCam_;
		    myCam_ = NULL;
	    }
        ROS_INFO("Cleaning up streams and piplines...");
    }

} // end of namespace
