#pragma once

#include "Grabber.h"
#include "Grabber_XIMEA.h"
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/shape_predictor.h>
#include <dlib/gui_widgets.h>

class Faces
{
    public:
      Faces();
      ~Faces();
      void getFaces(bool faces_flag, bool timing);
      bool getReady();
      void setPath(Grabber_XIMEA* grab, std::string path, bool _vis, bool _fit);
    protected:
      // DLIB
      dlib::frontal_face_detector detector;
      dlib::shape_predictor pose_model;
      dlib::image_window win;
      // ROS
      ros::NodeHandle n;
      ros::Publisher pub_f;
      // SELF
      Grabber_XIMEA* grabber;
      bool viz, fit, ready;
};
