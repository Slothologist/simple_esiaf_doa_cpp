#pragma once

#include "ros/ros.h"
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class Grabber
{
    public:
      Grabber();
      ~Grabber();
      void grabImage();
      ros::Time getTime();
      void setCapture(int _argc, const char *_argv[], int framerate);
      int getCamera();
      cv::Mat getImage();
      ros::Time timestamp;
    protected:
      cv::VideoCapture cap;
      cv::Mat frame;
      std::recursive_mutex mtx;
      int usingCamera;
};
