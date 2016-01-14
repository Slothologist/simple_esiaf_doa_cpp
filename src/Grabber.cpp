#include "Grabber.h"
#include <nmpt/NMPTUtils.h>
#include <iostream>
#include "ros/ros.h"
#include "m3api/xiApi.h"
#include "boost/date_time/posix_time/posix_time.hpp"


using namespace cv;
using namespace std;

Grabber::Grabber() {}
Grabber::~Grabber() {}

void Grabber::setCapture(int _argc, const char* _argv[], int framerate) {

    printf("OPENCV VERSION: %d.%d\n", CV_MAJOR_VERSION, CV_MINOR_VERSION);
    cv::Size imSize(320,240);
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
      boost::posix_time::ptime init = boost::posix_time::microsec_clock::local_time();
      if (cap.grab()){
        ros::Time frame_time = ros::Time::now();
        boost::posix_time::ptime re = boost::posix_time::microsec_clock::local_time();
        if (!cap.retrieve(source_frame)){
                cerr << "ERROR: failed to retrieve image, DROPPED ONE FRAME!\n";
                continue;
        }
        boost::posix_time::ptime c3 = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration cdiff3 = c3 - re;
        // cout << "[RETRIEVE] Time Consumption: " << cdiff3.total_milliseconds() << " ms" << std::endl;

        // Copy
        mtx.lock();
        output_frame = resized_frame.clone();
        timestamp = frame_time;
        mtx.unlock();
    }
    boost::posix_time::ptime c = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration cdiff = c - init;
    // cout << "[GRABBING] Time Consumption: " << cdiff.total_milliseconds() << " ms" << std::endl;
  }
}

cv::Mat Grabber::getImage(ros::Time *target_timestamp)
{
    mtx.lock();
    cv::Mat frame_copy = output_frame.clone();
    if (target_timestamp != NULL){
        *target_timestamp = timestamp;
    }
    mtx.unlock();
    return frame_copy;
}
