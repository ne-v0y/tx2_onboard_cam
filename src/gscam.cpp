/*
 * inference-101
 */

#include "gstCamera.h"

#include "glDisplay.h"
#include "glTexture.h"

#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#include "cudaNormalize.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "gscam_node.h"
#include "utils.h"

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

bool signal_recieved = false;

void sig_handler(int signo)
{
	if( signo == SIGINT )
	{
		printf("received SIGINT\n");
		signal_recieved = true;
	}
}

int main( int argc, char** argv )
{
	printf("gst-camera\n  args (%i):  ", argc);

	for( int i=0; i < argc; i++ )
		printf("%i [%s]  ", i, argv[i]);
		
	printf("\n");


    /*
     * start ROS handler
     */
    ros::init(argc, argv, "onBoardCamera");
    ros::NodeHandle nh, nh_private("~");
    ros::Publisher testing_publisher;
    testing_publisher = nh.advertise<sensor_msgs::Image>("/camera/tesing/raw", 1);

	if( signal(SIGINT, sig_handler) == SIG_ERR )
		printf("\ncan't catch SIGINT\n");

	/*
	 * create the camera device
	 */
	gstCamera* camera = gstCamera::Create();
	
	if( !camera )
	{
		printf("\ngst-camera:  failed to initialize video device\n");
		return 0;
	}
	
	printf("\ngst-camera:  successfully initialized video device\n");
	printf("    width:  %u\n", camera->GetWidth());
	printf("   height:  %u\n", camera->GetHeight());
	printf("    depth:  %u (bpp)\n", camera->GetPixelDepth());
	
	/*
	 * start streaming
	 */
	if( !camera->Open() )
	{
		printf("\ngst-camera:  failed to open camera for streaming\n");
		return 0;
	}
	
	printf("\ngst-camera:  camera open for streaming\n");
	
	
	while( !signal_recieved )
	{
		void* imgCPU  = NULL;
		void* imgCUDA = NULL;
		
		// get the latest frame
		if( !camera->Capture(&imgCPU, &imgCUDA, 1000) )
			printf("\ngst-camera:  failed to capture frame\n");
		else
			printf("gst-camera:  recieved new frame  CPU=0x%p  GPU=0x%p\n", imgCPU, imgCUDA);
		
		// convert from YUV to RGBA
		void* imgRGBA = NULL;

		if( !camera->ConvertRGBA(imgCUDA, &imgRGBA, true) )
			printf("gst-camera:  failed to convert from NV12 to RGBA\n");

		// rescale image pixel intensities
/*
		CUDA(cudaNormalizeRGBA((float4*)imgRGBA, make_float2(0.0f, 255.0f), 
						   (float4*)imgRGBA, make_float2(0.0f, 255.0f), 
 						   camera->GetWidth(), camera->GetHeight()));
*/  


        int img_height = helperlibs::Float2Int()(camera->GetHeight());
        int img_width = helperlibs::Float2Int()(camera->GetWidth());

        Mat test = Mat::zeros(img_height, img_width, CV_32FC4);
        helperlibs::Float42Mat(static_cast<float4*>(imgRGBA), test, img_width, img_height);
        //cudaMemcpy(test.data, imgRGBA, camera->GetWidth() * camera->GetHeight() * sizeof(float), cudaMemcpyDeviceToDevice);
        
        Mat out;
        test.convertTo(out, CV_8UC4);

        sensor_msgs::ImagePtr msg;
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", out).toImageMsg();
        ROS_INFO("Publishing to image topic");
        testing_publisher.publish(msg);

	} 
	
	
	/*
	 * shutdown the camera device
	 */
	if( camera != NULL )
	{
		delete camera;
		camera = NULL;
	}

	printf("gst-camera:  video device has been un-initialized.\n");
	printf("gst-camera:  this concludes the test of the video device.\n");
	return 0;
  
    // ros tick
    ros::spin();
}
