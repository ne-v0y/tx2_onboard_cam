# on_board_cam
ROS package that launches Jetson TX2 on-board camera  
Output topics:  
`/bottom/camera/image`  
`/bottom/camera_info`  
Built on top of [jetson-inference](https://github.com/dusty-nv/jetson-inference) gscamera.  

## Usage  
- `rosrun on_board_cam gscam` after successfully compiles under your catkin workspace  

## Compiling dependencies  
- ROS(built on Kinect, not tested on other distro yet)    
- OpenCV  
- CUDA Library  
- gstreamer  
