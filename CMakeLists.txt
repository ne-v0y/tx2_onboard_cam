cmake_minimum_required(VERSION 2.8.3)
project(on_board_cam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)

# find gstreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER QUIET gstreamer-0.10)
if(NOT GSTREAMER_FOUND)
  set(GSTREAMER_VERSION_1_x TRUE)
endif()
if(GSTREAMER_VERSION_1_x)
  message(STATUS "gst 1.0")
  pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
  pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
else()
  message(STATUS "gst 0.1")
  pkg_check_modules(GSTREAMER REQUIRED gstreamer-0.10)
  pkg_check_modules(GST_APP REQUIRED gstreamer-app-0.10)
endif()

#find glew
find_package(GLEW REQUIRED)

# find Qt5
find_package(Qt5Widgets REQUIRED)

# find cuda
find_package(CUDA)
set(
	CUDA_NVCC_FLAGS
	${CUDA_NVCC_FLAGS}; 
  -O3 
	-gencode arch=compute_53,code=sm_53
	-gencode arch=compute_62,code=sm_62
)
 
# find X11
find_package(X11 REQUIRED)

link_directories(${X11_LIBRARIES})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES on_board_cam
  CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########
include_directories(include/on_board_cam
  ${CUDA_INCLUDE_DIRS}
  ${X11_INCLUDE_DIR} 
  ${GLIB_INCLUDE_DIRS} 
  ${GLEW_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

cuda_add_library(internal_cuda 
  src/cudaFont.cu 
  src/cudaNormalize.cu 
  src/cudaOverlay.cu
  src/cudaResize.cu 
  src/cudaRGB.cu 
  src/cudaYUV-NV12.cu 
  src/cudaYUV-YUYV.cu 
  src/cudaYUV-YV12.cu
)

add_library(utils 
  src/commandLine.cpp
  src/loadImage.cpp
  src/glDisplay.cpp
  src/glTexture.cpp
  src/gstUtility.cpp
  src/gstCamera.cpp
  src/v4l2Camera.cpp
  src/utils.cpp
)
target_link_libraries(utils
  internal_cuda
  ${Boost_LIBRARIES}
)

add_executable(gscam src/gscam.cpp)
target_link_libraries(gscam
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  internal_cuda
  utils
  ${CUDA_LIBRARIES}
  ${GSTREAMER_LIBRARIES} 
  ${GST_APP_LIBRARIES}
  GL
  GLEW
  Qt5::Widgets
  X11
  Xext
)
