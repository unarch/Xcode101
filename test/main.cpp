#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using std::string;


int main(int argc, const char * argv[]) {
    string path = "";
    Mat image = imread(path);
    namedWindow("origin");
    imshow("origin", image);
    
    Mat gray;
    cvtColor(image, gray, COLOR_RGBA2GRAY);
    namedWindow("gray");
    imshow("gray", gray);
    waitKey(0);

    return 0;
}
