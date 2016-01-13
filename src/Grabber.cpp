#include "Grabber.h"
#include <nmpt/NMPTUtils.h>
#include <iostream>
#include "ros/ros.h"


using namespace cv;

Grabber::Grabber() {}
Grabber::~Grabber() {}

void Grabber::setCapture(int _argc, const char* _argv[], int framerate) {
    Size imSize(320,240);
    VideoCapture capture;
    cap = capture;
    usingCamera = NMPTUtils::getVideoCaptureFromCommandLineArgs(cap, _argc, _argv);
    if (!usingCamera--) {
        std::cout << "Sorry no static video file support for now" << std::endl;
        exit(1);
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH, imSize.width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, imSize.height);
    cap.set(CV_CAP_PROP_FPS, framerate);
}

int Grabber::getCamera(){
    return usingCamera;
}

void Grabber::grabImage()
{
  while(1) {
    if (cap.grab()){
        mtx.lock();
        //NOTE: access to timestamp AND frame has to be locked by mutex!
        timestamp = ros::Time::now();
        if (!cap.retrieve(frame)){
                printf("ERROR: failed to retrieve image\n");
        }
        mtx.unlock();
    }
  }
}

cv::Mat Grabber::getImage(ros::Time *target_timestamp)
{
    mtx.lock();
    cv::Mat frame_copy;
    frame_copy = frame.clone();
    if (target_timestamp != NULL){
        *target_timestamp = timestamp;
    }
    mtx.unlock();
    return frame_copy;
}
