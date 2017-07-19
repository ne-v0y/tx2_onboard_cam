// a series of helper functions

#include <utils.h>

using namespace cv;
using namespace std;

namespace helperlibs {

Mat Float42Mat(float4* src, Mat& mat, int width, int height) {

    if (src == NULL) return mat;

    for (int i = 0; i < width * height; i++) {
        int row = floor(i / width);
        int col = i % width;
        
        //printf("Row: %d Col: %d\n", row, col);

        mat.at<cv::Vec4f>(row, col)[0] = src[i].z;
        mat.at<cv::Vec4f>(row, col)[1] = src[i].y;
        mat.at<cv::Vec4f>(row, col)[2] = src[i].x;
        mat.at<cv::Vec4f>(row, col)[3] = 0;
    }
    
    cout << "Read image" << endl;
    return mat;
}

}
