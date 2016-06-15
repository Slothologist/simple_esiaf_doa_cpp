#pragma once

// STD
#include <iostream>
#include <sstream>
#include <string>

// SELF
#include "Grabber.h"
#include "Grabber_ROS.h"

// DLIB
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/shape_predictor.h>
#include <dlib/gui_widgets.h>

class Faces {
public:
    Faces(std::string topic);
    ~Faces();
    void getFaces(bool faces_flag, bool timing, int throttle);
    void setPath(Grabber *grab, std::string path, bool _vis, bool _fit);
    void setPathROS(Grabber_ROS *grab, std::string path, bool _vis, bool _fit);

    boost::mutex connect_cb_mutex_;

protected:

    // DLIB
    dlib::frontal_face_detector detector;
    dlib::shape_predictor pose_model;
    dlib::image_window win;

    // ROS
    ros::NodeHandle n;
    ros::Publisher pub_f;

    // SELF
    Grabber *grabber;
    Grabber_ROS *grabber_ros;
    
    bool viz, fit, is_ximea, is_ros, is_native;

private:
    // Subscriber handling
    bool has_subscribers;
    ros::SubscriberStatusCallback connect_cb;
    ros::NodeHandle *nh;
    boost::mutex connect_cb_mutex_f_;
    void connectCb();
};
