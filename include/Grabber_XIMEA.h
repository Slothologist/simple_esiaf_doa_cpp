#pragma once

#include "ros/ros.h"
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class Grabber_XIMEA
{
    public:
      Grabber_XIMEA();
      ~Grabber_XIMEA();
      void grabImage();
      int getCamera();
      void setCapture(int _argc, const char *_argv[], int framerate);
      cv::Mat getImage(ros::Time *timestamp);
    protected:
      ros::Time timestamp;
      cv::VideoCapture cap;
      cv::Mat frame;
      std::recursive_mutex mtx;
      cv::Mat source_frame;
      cv::Mat resized_frame;
      cv::Mat output_frame;
      cv::Rect cutout_roi;
      int usingCamera;

};
