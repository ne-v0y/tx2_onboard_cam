/*
 * inference-101
 */
#include <signal.h>
#include <ros/ros.h>

#include <gscam.h>


bool error_signal_recieved_ = false;

void sig_handler(int signo)
{
	if( signo == SIGINT )
	{
		printf("received SIGINT\n");
		error_signal_recieved_ = true;
	}
}

int main( int argc, char** argv )
{
	if( signal(SIGINT, sig_handler) == SIG_ERR )
		ROS_WARN("can't catch SIGINT");

    ros::init(argc, argv, "gscam_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    au_core::GSCam gscam(nh, private_nh);

    while(ros::ok() && !error_signal_recieved_)
    {
        gscam.run();
    }
    
    gscam.close();

    return 0;
}

