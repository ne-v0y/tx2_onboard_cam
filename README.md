# on_board_cam
ROS package that launches Jetson TX2 on-board camera  
Output topics: `/bottom/camera/image`  
               `/bottom/camera_info`  

## Usage  
- `rosrun on_board_cam gscam` after successfully compiles under your catkin workspace  

## Compiling dependencies  
- ROS(built on Kinect, not tested on other distro yet)    
- OpenCV  
- CUDA Library  
- gstreamer  
