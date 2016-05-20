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
    // void setPathXimea(Grabber_XIMEA *grab, std::string path, bool _vis, bool _fit);
    void setPath(Grabber *grab, std::string path, bool _vis, bool _fit);
    void setPathROS(Grabber_ROS *grab, std::string path, bool _vis, bool _fit);
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
    // Grabber_XIMEA *grabber_x;

    bool viz, fit, is_ximea, is_ros, is_native;
};
