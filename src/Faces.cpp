#include "Faces.h"

// ROS
#include "ros/ros.h"
#include "people_msgs/People.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"

// BOOST
#include "boost/date_time/posix_time/posix_time.hpp"

// DLIB
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>


using namespace std;

Faces::Faces(){
    ready = false;
    this->pub_f = n.advertise<people_msgs::People>("robotgazetools/faces", 10);
}

Faces::~Faces(){}

bool Faces::getReady() {
    return ready;
}

void Faces::setPath(Grabber_XIMEA* grab, string path, bool _vis, bool _fit)
{
    grabber = grab;
    viz = _vis;
    fit = _fit;
    detector = dlib::get_frontal_face_detector();

    try {
        dlib::deserialize(path) >> pose_model;
        ready = true;
    } catch(dlib::serialization_error& e) {
        cout << "You need dlib's default face landmarking model file to run this example." << endl;
        cout << "You can get it from the following URL: " << endl;
        cout << "http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2" << endl;
        cout << endl << e.what() << endl;
        exit(1);
    }
}


void Faces::getFaces(bool faces_flag, bool timing)
{

    ros::Time old = ros::Time::now();

    while(1) {

        if (!faces_flag) {
            usleep(5000);
            return;
        }

        boost::posix_time::ptime init = boost::posix_time::microsec_clock::local_time();

        ros::Time frame_timestamp;
        cv::Mat im = grabber->getImage(&frame_timestamp);

        // If no image has been grabbed yet...wait.
        if (im.rows == 0 || im.cols == 0) {
            cout << "Faces: waiting for next image to be grabbed..." << endl;
            continue;
        }

        std_msgs::Header h;
        h.stamp = frame_timestamp;
        h.frame_id = "1";

        if (h.stamp <= old){
            usleep(1000);
            continue;
        }
        old = h.stamp;

        // ROS MSGS
        people_msgs::People people_msg;
        people_msgs::Person person_msg;

        dlib::cv_image<dlib::bgr_pixel> cimg(im);

        // UPSCALE IMAGE
        // dlib::pyramid_up(cimg);

        // Detect faces
        std::vector<dlib::rectangle> faces = detector(cimg);

        // Find the pose of each face.
        std::vector<dlib::full_object_detection> shapes;
        if(fit) {
            for (unsigned long i = 0; i < faces.size(); ++i) {
                shapes.push_back(pose_model(cimg, faces[i]));
            }
        }

        for(unsigned long i = 0; i < faces.size(); ++i) {
            // cout << "left: " << faces[i].left() << " top: " << faces[i].top() << endl;
            // cout << "right: " << faces[i].right() << " bottom: " << faces[i].bottom() << endl;
            person_msg.name = "unknown";
            person_msg.reliability = 0.0;
            geometry_msgs::Point p;
            double mid_x = (faces[i].left() + faces[i].right())/2.0;
            double mid_y = (faces[i].top() + faces[i].bottom())/2.0;
            p.x = mid_x;
            p.y = mid_y;
            p.z = faces[i].right() - faces[i].left();
            person_msg.position = p;
            people_msg.people.push_back(person_msg);
        }
        if(people_msg.people.size() > 0){
            people_msg.header = h;
            pub_f.publish(people_msg);
        }
        // Display it all on the screen
        if (viz) {
            win.set_title("Simple Robot Gaze Tools || DLIB");
            win.clear_overlay();
            win.set_image(cimg);
            win.add_overlay(faces, dlib::rgb_pixel(255,0,0));
            if(fit) {
                win.add_overlay(dlib::render_face_detections(shapes));
            }
        }

        if(timing) {
            boost::posix_time::ptime c = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration cdiff = c - init;
            cout << "[FACES] Time Consumption: " << cdiff.total_milliseconds() << " ms" << std::endl;
        }
    }
}
