#ifndef _GENERAL_UTILS_H__
#define _GENERAL_UTILS_H__

#include <iostream>
#include <cmath>

#include <opencv2/core/core.hpp>
#include "cudaUtility.h"

#include <sensor_msgs/Image.h>

#include <boost/numeric/conversion/converter.hpp>

namespace helperlibs {

void Float42Mat(float4* src, cv::Mat& mat, int width, int height);

void Float42SensorImg(float4* src, sensor_msgs::ImagePtr &img, std::string encoding, bool is_bigendian);

typedef boost::numeric::converter<int, float> Float2Int;

} // end of namespace

#endif
