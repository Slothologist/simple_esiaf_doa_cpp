#pragma once

// ROS
#include <ros/time.h>

// STD
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <mutex>

// BOOST
#include <boost/shared_ptr.hpp>

// RSC
#include <rsc/misc/langutils.h>
#include <rsc/threading/SynchronizedQueue.h>

// RSB
#include <rsb/util/EventQueuePushHandler.h>
#include <rsb/Listener.h>
#include <rsb/MetaData.h>

// CV
#include <opencv/cv.h>
#include <opencv/highgui.h>

// RST
#include <rst/converters/opencv/IplImageConverter.h>

class Grabber_RSB {

public:
    Grabber_RSB(bool timing, int width, int height, std::string scope, std::string host, bool socket_input);
    ~Grabber_RSB();
    void getImage(ros::Time *target_timestamp, cv::Mat *image);
    void grab();
    bool timing;
private:
    typedef rsc::threading::SynchronizedQueue<rsb::EventPtr> ImageQueue;
    typedef boost::shared_ptr<ImageQueue> ImageQueuePtr;
    typedef rsb::util::EventQueuePushHandler ImageHandler;
    typedef boost::shared_ptr<ImageHandler> ImageHandlerPtr;

    ImageQueuePtr imageQueue;
    ImageHandlerPtr imageHandler;

    rsb::ListenerPtr imageListener;
    rsb::MetaData imageMetaData;
    std::recursive_mutex mtx;
    cv::Mat output_frame;
    cv::Mat source_frame;
    ros::Time timestamp;
    int usingCamera, width, height;
};
