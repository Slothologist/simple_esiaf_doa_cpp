#pragma once

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// STD
#include <string>
#include <iostream>
#include <sstream>
#include <mutex>

// CV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class Grabber_ROS {

public:
    Grabber_ROS(bool timing, int width, int height, std::string scope);
    ~Grabber_ROS();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void getImage(ros::Time *timestamp, cv::Mat *mat);
    int  getCamera();
    bool timing;
private:
    ros::NodeHandle node_handle_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    std::recursive_mutex mtx;
    cv::Mat output_frame;
    cv::Mat source_frame;
    ros::Time timestamp;
    int usingCamera, width, height;
};

