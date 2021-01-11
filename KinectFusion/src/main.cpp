#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

using namespace std;
using namespace cv;

int main(int, char**){
    int a = 2;
    std::cout << a << std::endl;
    
    VideoCapture cap(0); 
    if(!cap.isOpened())
        return -1;
    
    for(;;){
        Mat frame;
        cap >> frame;
        imshow("Webcam Video", frame);
        if(waitKey(30) >= 0) break;
    }
    return 0;
}