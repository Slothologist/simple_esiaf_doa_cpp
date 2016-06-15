#include "Faces.h"

// ROS
#include "ros/ros.h"
#include "people_msgs/People.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"

// BOOST
#include "boost/date_time/posix_time/posix_time.hpp"
//#include <boost/program_options.hpp>

// DLIB
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>

using namespace std;

Faces::Faces(std::string topic) {
    ros::SubscriberStatusCallback connect_cb = boost::bind(&Faces::connectCb, this);
    boost::lock_guard<boost::mutex> lock(connect_cb_mutex_f_);
    this->pub_f = n.advertise<people_msgs::People>(topic+"/faces", 10, connect_cb, connect_cb);
}

Faces::~Faces() { }

/*
void Faces::setPathXimea(Grabber_XIMEA *grab, string path, bool _vis, bool _fit) {
    grabber_x = grab;
    viz = _vis;
    fit = _fit;
    is_ximea = true;
    is_ros = false;
    is_native = false;
    detector = dlib::get_frontal_face_detector();

    try {
        dlib::deserialize(path) >> pose_model;
    } catch (dlib::serialization_error &e) {
        cout << "You need dlib's default face landmarking model file to run this example." << endl;
        cout << "You can get it from the following URL: " << endl;
        cout << "http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2" << endl;
        cout << endl << e.what() << endl;
        exit(1);
    }
}
*/

void Faces::setPath(Grabber *grab, string path, bool _vis, bool _fit) {
    grabber = grab;
    viz = _vis;
    fit = _fit;
    is_ximea = false;
    is_ros = false;
    is_native = true;
    detector = dlib::get_frontal_face_detector();

    try {
        dlib::deserialize(path) >> pose_model;
    } catch (dlib::serialization_error &e) {
        cout << "You need dlib's default face landmarking model file to run this example." << endl;
        cout << "You can get it from the following URL: " << endl;
        cout << "http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2" << endl;
        cout << endl << e.what() << endl;
        exit(1);
    }
}

void Faces::setPathROS(Grabber_ROS *grab, string path, bool _vis, bool _fit) {
    grabber_ros = grab;
    viz = _vis;
    fit = _fit;
    is_ximea = false;
    is_ros = true;
    is_native = false;
    detector = dlib::get_frontal_face_detector();

    try {
        dlib::deserialize(path) >> pose_model;
    } catch (dlib::serialization_error &e) {
        cout << "You need dlib's default face landmarking model file to run this example." << endl;
        cout << "You can get it from the following URL: " << endl;
        cout << "http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2" << endl;
        cout << endl << e.what() << endl;
        exit(1);
    }
}

void Faces::getFaces(bool faces_flag, bool timing, int throttle) {

    ros::Time start = ros::Time::now();
    ros::Time last_frame_timestamp = ros::Time::now();

    while (true) {

        if (!faces_flag) {
            usleep(5000);
            return;
        }

        boost::posix_time::ptime init = boost::posix_time::microsec_clock::local_time();

        ros::Time frame_timestamp;
        cv::Mat im;

        // if (is_ximea) {
        //     grabber_x->getImage(&frame_timestamp, &im);
        // }

        if (is_ros) {
            grabber_ros->getImage(&frame_timestamp, &im);
        }

        if (is_native) {
            grabber->getImage(&frame_timestamp, &im);
        }

        // If no image has been grabbed yet...wait.
        if (im.rows == 0 || im.cols == 0) {
            // cout << "[Faces] waiting for next image to be grabbed..." << endl;
            usleep(1000);
            continue;
        }

        std_msgs::Header h;
        h.stamp = frame_timestamp;
        h.frame_id = "0";

        if (h.stamp <= start || last_frame_timestamp == frame_timestamp) {
            // cout << "[Faces] no new frame, continue..." << endl;
            usleep(1000);
            continue;
        }
        start = h.stamp;
        last_frame_timestamp = frame_timestamp;

        // ROS MSGS
        people_msgs::People people_msg;
        people_msgs::Person person_msg;

        dlib::cv_image<dlib::bgr_pixel> cimg(im);

        // Detect faces
        std::vector<dlib::rectangle> faces = detector(cimg);

        // Find the pose of each face.
        std::vector<dlib::full_object_detection> shapes;
        if (fit) {
            for (unsigned long i = 0; i < faces.size(); ++i) {
                shapes.push_back(pose_model(cimg, faces[i]));
            }
        }

        for (unsigned long i = 0; i < faces.size(); ++i) {
            person_msg.name = "unknown";
            person_msg.reliability = 0.0;
            geometry_msgs::Point p;
            double mid_x = (faces[i].left() + faces[i].right()) / 2.0;
            double mid_y = (faces[i].top() + faces[i].bottom()) / 2.0;
            p.x = mid_x;
            p.y = mid_y;
            p.z = faces[i].right() - faces[i].left();
            person_msg.position = p;
            people_msg.people.push_back(person_msg);
        }
        if (people_msg.people.size() > 0) {
            people_msg.header = h;
            pub_f.publish(people_msg);
        }

        // Display it all on the screen
        if (viz) {
            win.set_title("Simple Robot Gaze Tools || DLIB");
            win.clear_overlay();
            win.set_image(cimg);
            win.add_overlay(faces, dlib::rgb_pixel(255, 0, 0));
            if (fit) {
                win.add_overlay(dlib::render_face_detections(shapes));
            }
        }

        usleep(throttle);

        if (timing) {
            boost::posix_time::ptime c = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration cdiff = c - init;
            cout << "[FACES] Time Consumption: " << cdiff.total_milliseconds() << " ms" << std::endl;
        }
    }
}

void Faces::connectCb() {
  boost::lock_guard<boost::mutex> lock(connect_cb_mutex_f_);
  ROS_INFO_STREAM("pub_faces has " << pub_f.getNumSubscribers() << " susbcribers");
  has_subscribers = (pub_f.getNumSubscribers() != 0);
  if (!has_subscribers) {
    ROS_INFO_STREAM("unsubscribing image input topic for faces");
    grabber_ros->stop();
  } else {
    // subscribe input
    ROS_INFO_STREAM("(re)subscribing image input topic for faces");
    grabber_ros->start();
  }
}

