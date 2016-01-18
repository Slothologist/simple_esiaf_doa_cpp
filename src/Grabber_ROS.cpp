#include <ros/ros.h>
#include "Grabber_ROS.h"
#include "boost/date_time/posix_time/posix_time.hpp"

Grabber_ROS::Grabber_ROS(bool timing_flag) : it_(node_handle_) {
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &Grabber_ROS::imageCallback, this);
    usingCamera = 1;
    timing = timing_flag;
    // View Input Image (optional)
    cv::namedWindow("ROS INPUT STREAM");
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

    ros::Time frame_time = ros::Time::now();

    // Copy
    mtx.lock();
    output_frame = cv_ptr->image.clone();
    timestamp = frame_time;
    mtx.unlock();

    // View Input Image (optional)
    cv::imshow("ROS INPUT", cv_ptr->image);
    cv::waitKey(1);

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
    cv::Mat frame_copy = output_frame.clone();
    *mat = frame_copy;
    if (target_timestamp != NULL) {
        *target_timestamp = timestamp;
    }
    mtx.unlock();
}
