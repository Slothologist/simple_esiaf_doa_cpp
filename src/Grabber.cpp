#include "Grabber.h"
#include <nmpt/NMPTUtils.h>
#include <iostream>
#include "ros/ros.h"
#include "boost/date_time/posix_time/posix_time.hpp"


using namespace cv;
using namespace std;

Grabber::Grabber() { }
Grabber::~Grabber() { }

void Grabber::setCapture(int _argc, const char *_argv[], int framerate, bool timing_flag, int i_width, int i_height) {

    printf("OPENCV VERSION: %d.%d\n", CV_MAJOR_VERSION, CV_MINOR_VERSION);

    timing = timing_flag;
    height = i_height;
    width = i_width;
    VideoCapture capture;
    cap = capture;

    cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    cap.set(CV_CAP_PROP_FPS, framerate);

    usingCamera = NMPTUtils::getVideoCaptureFromCommandLineArgs(cap, _argc, _argv);

    if (!usingCamera--) {
        std::cout << ">>> Sorry no static video file support for now" << std::endl;
        exit(EXIT_SUCCESS);
    }

    if (!cap.isOpened()) {
        cerr << ">>> ERROR: capture is NULL, no camera device found.\n";
        exit(EXIT_FAILURE);
    }
}

int Grabber::getCamera() {
    return usingCamera;
}

void Grabber::grabImage() {
    while (1) {
        boost::posix_time::ptime init = boost::posix_time::microsec_clock::local_time();

        if (cap.grab()) {
            ros::Time frame_time = ros::Time::now();
            boost::posix_time::ptime re = boost::posix_time::microsec_clock::local_time();
            if (!cap.retrieve(source_frame)) {
                cerr << "ERROR: failed to retrieve image, DROPPED ONE FRAME!\n";
                continue;
            }

            cv::Size size(width,height);

            // Copy
            mtx.lock();
            if (source_frame.rows > 0 && source_frame.cols > 0 && source_frame.rows != width && source_frame.cols != height) {
                 cv::resize(source_frame, output_frame, size);
            } else {
                output_frame = source_frame;
            }
            timestamp = frame_time;
            mtx.unlock();
        }

        if(timing) {
            boost::posix_time::ptime c = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration cdiff = c - init;
            std::cout << "[GRABBING NATIVE] Time Consumption: " << cdiff.total_milliseconds() << " ms" << std::endl;
        }
    }
}

void Grabber::getImage(ros::Time *target_timestamp, cv::Mat *mat) {
    mtx.lock();
    *mat = output_frame;
    if (target_timestamp != NULL) {
        *target_timestamp = timestamp;
    }
    mtx.unlock();
}
