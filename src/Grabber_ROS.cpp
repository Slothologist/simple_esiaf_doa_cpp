// ROS
#include <ros/ros.h>

// SELF
#include "Grabber_ROS.h"

// STD
#include <iostream>
#include <sstream>
#include <string>

// BOOST
#include "boost/date_time/posix_time/posix_time.hpp"

Grabber_ROS::Grabber_ROS(bool timing_flag, int i_width, int i_height, std::string scope) : it_(node_handle_) {
    usingCamera = 1;
    timing = timing_flag;
    width = i_width;
    height = i_height;
    scope_ = scope;
    image_sub_ = it_.subscribe(scope, 5, &Grabber_ROS::imageCallback, this);
}

Grabber_ROS::~Grabber_ROS() { }

void Grabber_ROS::imageCallback(const sensor_msgs::ImageConstPtr &msg) {

    boost::posix_time::ptime init = boost::posix_time::microsec_clock::local_time();

    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR(">>> CV_BRIDGE exception: %s", e.what());
        return;
    }

    cv::Size size(width, height);

    // Copy

    ros::Time frame_time = msg->header.stamp;
    timestamp = frame_time;
    mtx.lock();
    source_frame = cv_ptr->image;
    if (source_frame.rows != width || source_frame.cols != height) {
         cv::resize(source_frame, output_frame, size);
    } else {
        output_frame = source_frame;
    }
    mtx.unlock();

    if(timing) {
        boost::posix_time::ptime c = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration cdiff = c - init;
        std::cout << "[GRABBING ROS] Time Consumption: " << cdiff.total_milliseconds() << " ms" << std::endl;
    }

}

int Grabber_ROS::getCamera() {
    return usingCamera;
}

void Grabber_ROS::getImage(ros::Time *target_timestamp, cv::Mat *mat) {
    mtx.lock();
    *mat = output_frame;
    if (target_timestamp != NULL) {
        *target_timestamp = timestamp;
    }
    mtx.unlock();
}

void Grabber_ROS::start() {
  std::cout << ">>> Subscribing to " << scope_ << std::endl;
  image_sub_ = it_.subscribe(scope_, 5, &Grabber_ROS::imageCallback, this);
}

void Grabber_ROS::stop() {
  std::cout << ">>> Shutdown for " << scope_ << std::endl;
  image_sub_.shutdown();
}
