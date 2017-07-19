#ifndef _GENERAL_UTILS_H__
#define _GENERAL_UTILS_H__

#include <iostream>
#include <cmath>

#include <opencv2/core/core.hpp>
#include "cudaUtility.h"

#include <boost/numeric/conversion/converter.hpp>

namespace helperlibs {

// take an array of float4 and transform to height * width * 4
cv::Mat Float42Mat(float4* src, cv::Mat& mat, int width, int height);

typedef boost::numeric::converter<int, float> Float2Int;

} // end of namespace

#endif
