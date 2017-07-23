// a series of helper functions

#include <utils.h>

using namespace cv;
using namespace std;

namespace helperlibs {

// convert src image to OpenCV Matrix 
// TODO:use memcopy?
void Float42Mat(float4* src, Mat& mat, int width, int height) {

    if (src == NULL) exit(0);

    for (int i = 0; i < width * height; i++) {
        int row = floor(i / width);
        int col = i % width;
        
        //printf("Row: %d Col: %d\n", row, col);

        mat.at<cv::Vec3f>(row, col)[0] = src[i].z;
        mat.at<cv::Vec3f>(row, col)[1] = src[i].y;
        mat.at<cv::Vec3f>(row, col)[2] = src[i].x;
    }
    
}

// convert src image byte to sensor image format msg
void Float42SensorImg(float4* src, sensor_msgs::ImagePtr &img, std::string encoding, bool is_bigendian, int w, int h) {
    // TODO
}

} // end of namespace
