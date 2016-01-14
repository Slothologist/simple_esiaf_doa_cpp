#include "Grabber_XIMEA.h"
#include "Grabber.h"
#include <nmpt/NMPTUtils.h>
#include <iostream>
#include "ros/ros.h"
#include "m3api/xiApi.h"
#include "boost/date_time/posix_time/posix_time.hpp"


using namespace cv;
using namespace std;

Grabber_XIMEA::Grabber_XIMEA() {}
Grabber_XIMEA::~Grabber_XIMEA() {}

void Grabber_XIMEA::setCapture(int _argc, const char* _argv[], int framerate) {

    printf("OPENCV VERSION: %d.%d\n", CV_MAJOR_VERSION, CV_MINOR_VERSION);
    printf("xiapi %d\n", CV_CAP_XIAPI );

    VideoCapture capture(CV_CAP_XIAPI);

    cap = capture;

    if ( !cap.isOpened() ) {
        cerr << "ERROR: capture is NULL, no ximea cam found. Is Opencv built with ximea api??\n";
        exit(EXIT_FAILURE);
    }

    usingCamera = 1;

    // XIMEA settings
    cap.set( CV_CAP_PROP_XI_AE_MAX_LIMIT, 20*1000);
    cap.set( CV_CAP_PROP_XI_RECENT_FRAME, 1);
    cap.set( CV_CAP_PROP_XI_BUFFERS_QUEUE_SIZE, 3);
    cap.set( CV_CAP_PROP_XI_AEAG, 1);
    cap.set( CV_CAP_PROP_XI_ACQ_TIMING_MODE, XI_ACQ_TIMING_MODE_FRAME_RATE);
    cap.set( CV_CAP_PROP_FPS, framerate);

    unsigned int source_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    unsigned int source_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    cout << "Source height " << source_height << " source width " << source_width << endl;

    // Code supports 2048x2048 and 2048x1080 cameras:
    if ((source_width == 2040) && (source_height == 1080)){
        //2048x1080 sensor MQ022
        cout << "Detected MQ022 camera\n";
        unsigned int target_width = source_height * (4.0/3.0);
        unsigned int image_cutoff_w = (source_width - target_width)/2;
        cout << target_width << "\n";
        cout << image_cutoff_w << "\n";

        cutout_roi = cv::Rect(image_cutoff_w/2, 0, target_width , source_height);
    }else if ((source_width == 2040) && (source_height == 2040)){
        //2048x2048 sensor MQ042
        cout << "Detected MQ042 camera\n";
        unsigned int resize_ratio = source_width  * (3.0/4.0);
        unsigned int image_cutoff_h = source_height-resize_ratio;
        cutout_roi = cv::Rect(0, image_cutoff_h/2, source_width , source_height-image_cutoff_h);
    } else {
        cerr << "WARNING: no supported camera detected (only MQ022 and MQ042 are supported)" << endl;
        exit(EXIT_FAILURE);
    }
    cout << "Resize roi is " << cutout_roi << "\n";
}

int Grabber_XIMEA::getCamera(){
    return usingCamera;
}

void Grabber_XIMEA::grabImage()
{
  cv::Size imSize(320,240);
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
        // cout << "[RETR] Time Consumption: " << cdiff3.total_milliseconds() << " ms" << std::endl;


        boost::posix_time::ptime r = boost::posix_time::microsec_clock::local_time();

        // Crop
        Mat cropped = source_frame(cutout_roi);

        // Resize
        Mat resized;
        resize(cropped, resized_frame, imSize, INTER_NEAREST);

        boost::posix_time::ptime c2 = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration cdiff2 = c2 - r;
        // cout << "[RESIZE +CROP] Time Consumption: " << cdiff2.total_milliseconds() << " ms" << std::endl;

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

cv::Mat Grabber_XIMEA::getImage(ros::Time *target_timestamp)
{
    mtx.lock();
    cv::Mat frame_copy = output_frame.clone();
    if (target_timestamp != NULL){
        *target_timestamp = timestamp;
    }
    mtx.unlock();
    return frame_copy;
}

