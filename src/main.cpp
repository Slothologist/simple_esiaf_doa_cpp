/*
 *  simple_robot_gaze_tools
 *
 *  Created by Nicholas Butko on 11/15/10.
 *  Used and extended by Florian Lier on December 2015
 *  Copyright 2010 UC San Diego. All rights reserved.
 *  Copyright 2015 Bielefeld University. All rights reserved.
 *
 */


// CV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <sstream>
#include <thread>

// Boost
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/tokenizer.hpp>
#include <boost/token_functions.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

// NMPT
#include <nmpt/BlockTimer.h>
#include <nmpt/FastSalience.h>
#include <nmpt/LQRPointTracker.h>
#include <nmpt/NMPTUtils.h>

// ROS
#include "ros/ros.h"
#include "people_msgs/People.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"

// DLIB
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>

using namespace std;
using namespace cv;
using namespace boost;
using namespace boost::program_options;
using namespace boost::posix_time;


// Find Faces and Publish
class Faces
{
    public:
      Faces();
      ~Faces();
      void getFaces(cv::Mat img, std_msgs::Header header);
      void setPath(String path, bool _vis, bool _fit);

    protected:
      // DLIB
      dlib::frontal_face_detector detector;
      dlib::shape_predictor pose_model;
      dlib::image_window win;
      // ROS
      ros::NodeHandle n;
      ros::Publisher pub_f;
      // SELF
      bool viz, fit;
};

Faces::Faces(){
    this->pub_f = n.advertise<people_msgs::People>("robotgazetools/faces", 10);
}
Faces::~Faces(){}

void Faces::setPath(String path, bool _vis, bool _fit)
{
    viz = _vis;
    fit = _fit;

    detector = dlib::get_frontal_face_detector();
    try {
        dlib::deserialize(path) >> pose_model;
    } catch(dlib::serialization_error& e) {
        cout << "You need dlib's default face landmarking model file to run this example." << endl;
        cout << "You can get it from the following URL: " << endl;
        cout << "http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2" << endl;
        cout << endl << e.what() << endl;
        exit(1);
    }
}


void Faces::getFaces(cv::Mat _img, std_msgs::Header header)
{
    // ROS MSGS
    people_msgs::People people_msg;
    people_msgs::Person person_msg;

    dlib::cv_image<dlib::bgr_pixel> cimg(_img);

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
        people_msg.header = header;
        pub_f.publish(people_msg);
    }
    // Display it all on the screen
    if (viz) {
        win.clear_overlay();
        win.set_image(cimg);
        win.add_overlay(faces, dlib::rgb_pixel(255,0,0));
        if(fit) {
            win.add_overlay(render_face_detections(shapes));
        }
    }
}

// Find Most Salient Point and Publish
class Saliency
{
    public:
      Saliency();
      ~Saliency();
      void getSaliency(cv::Mat img, std_msgs::Header header);
      void setup(int camera, bool _vis);
    protected:
      // NMPT
      BlockTimer bt;
      cv::Mat viz, sal;
      int usingCamera;
      // ROS
      ros::NodeHandle n;
      ros::Publisher pub_s;
      // SELF
      bool vizu;
      // NMPT-2
      /**
       * @param numtemporal Number of timescales of Difference of Expontential filters to track.
       * @param numspatial Number of sizes of Difference of Box filters to use.
       * @param firsttau Exponential Falloff parameter for the first Difference of Exponentials scale. Lower numbers
       * give slower falloff. Must be greater than 0.
       * @param firstrad Radius of smallest Difference of Boxes filter center. The diameter of the box is 2*rad+1, so
       * the smallest allowed first radius is 0.
       */
      // Default: FastSalience salTracker;
      FastSalience salTracker;
      vector<double> lqrpt{2,.5};
      // Deafault: salientSpot{2};
      LQRPointTracker salientSpot{2, 1.0, 0,.015};
};

Saliency::Saliency() {
    pub_s = n.advertise<geometry_msgs::PointStamped>("robotgazetools/saliency", 10);
}

Saliency::~Saliency(){}

void Saliency::setup(int camera, bool _vis) {
    cout << ">>> Setting up Saliency..." << endl;
    usingCamera = camera;
    bt.blockRestart(1);
    salientSpot.setTrackerTarget(lqrpt);
    salTracker.setUseDoEFeatures(1);
    vizu = _vis;
    cout << ">>> Done!" << endl;
}

void Saliency::getSaliency(cv::Mat im, std_msgs::Header header)
{
    double saltime, tottime;

    // Time me
    bt.blockRestart(0);

    viz.create(im.rows, im.cols*2, CV_32FC3);

    vector<KeyPoint> pts;
    salTracker.detect(im, pts);
    saltime = bt.getCurrTime(0) ;

    salTracker.getSalImage(sal);

    double min, max;
    Point minloc, maxloc;
    minMaxLoc(sal, &min, &max, &minloc, &maxloc);

    lqrpt[0] = maxloc.x*1.0 / sal.cols;
    lqrpt[1] = maxloc.y*1.0 / sal.rows;

    salientSpot.setTrackerTarget(lqrpt);

    Mat vizRect = viz(Rect(im.cols,0,im.cols, im.rows));
    cvtColor(sal, vizRect, CV_GRAY2BGR);

    vizRect = viz(Rect(0, 0, im.cols, im.rows));
    im.convertTo(vizRect,CV_32F, 1./256.);

    for (size_t i = 0; i < pts.size(); i++) {
        circle(vizRect, pts[i].pt, 2, CV_RGB(0,255,0));
    }

    salientSpot.updateTrackerPosition();
    lqrpt = salientSpot.getCurrentPosition();

    circle(vizRect, Point(lqrpt[0]*sal.cols, lqrpt[1]*sal.rows), 6, CV_RGB(0,0,255));
    circle(vizRect, Point(lqrpt[0]*sal.cols, lqrpt[1]*sal.rows), 5, CV_RGB(0,0,255));
    circle(vizRect, Point(lqrpt[0]*sal.cols, lqrpt[1]*sal.rows), 4, CV_RGB(255,255,0));
    circle(vizRect, Point(lqrpt[0]*sal.cols, lqrpt[1]*sal.rows), 3, CV_RGB(255,255,0));

    // cout << "Most Salient Point: X " << lqrpt[0]*sal.cols << " Y " << lqrpt[1]*sal.rows << endl;
    geometry_msgs::PointStamped ps;
    geometry_msgs::Point p;
    double mid_x = lqrpt[0]*sal.cols;
    double mid_y = lqrpt[1]*sal.rows;
    p.x = mid_x;
    p.y = mid_y;
    p.z = pts.size();
    ps.point = p;
    ps.header = header;
    pub_s.publish(ps);

    tottime = bt.getCurrTime(1);

    // Stop Timer
    bt.blockRestart(1);

    stringstream text;
    text << "FastSUN: " << (int)(saltime*1000) << " ms ; Total: " << (int)(tottime*1000) << " ms.";

    putText(viz, text.str(), Point(20,20), FONT_HERSHEY_SIMPLEX, .33, Scalar(255,0,255));\

    if(vizu) {
        imshow("SRG-Tools || NMPT Salience || Press Q to Quit", viz);
    }
}


int main (int argc, char * const argv[])
{
    // Global toogle/config
    String dlib_path = "None";
    bool faces_flag = false;
    bool viz_flag = false;
    bool saliency_flag = false;
    bool fit_flag = false;
    bool timing_flag = false;

    // Programm options
    try {

        options_description general("general options");
        general.add_options()
            ("help", "Show this help")
            ("version", "0.1")
            ;

        options_description dlib("dlib options");
        dlib.add_options()
            ("dlib", value<string>(), "the dlib pose model path")
            ;

        options_description saliency("saliency detection options");
        saliency.add_options()
            ("saliency", value<string>(), "detect most salient point ON|OFF")
            ;

        options_description faces("face detection options");
        faces.add_options()
            ("faces", value<string>(), "detect faces ON|OFF")
            ;

        options_description viz("visualization options");
        viz.add_options()
            ("viz", value<string>(), "visulatization ON|OFF")
            ;

        options_description fit("fitting options");
        fit.add_options()
            ("fit", value<string>(), "fitting ON|OFF")
            ;

        options_description timing("timing options");
        timing.add_options()
            ("timing", value<string>(), "timing ON|OFF")
            ;

        options_description all("Allowed options");
        all.add(general).add(dlib).add(saliency).add(faces).add(viz).add(fit).add(timing);

        options_description visible("Allowed options");
        visible.add(general).add(dlib).add(saliency).add(faces).add(viz).add(fit).add(timing);

        variables_map vm;

        store(parse_command_line(argc, argv, all), vm);

        if (vm.count("help"))
        {
            cout << visible;
            return 0;
        }

        if (vm.count("faces")) {
            const string& s = vm["faces"].as<string>();
            if(s=="ON") {
                cout << ">>> Face detection is: " << s << "\n";
                faces_flag = true;
                if (vm.count("dlib")) {
                    const string& s = vm["dlib"].as<string>();
                    dlib_path = s;
                    cout << ">>> DLIB pose model found!" << "\n";
                } else {
                    cout << ">>> ERROR: DLIB pose model path NOT set \n";
                    faces_flag = false;
                    return 0;
                }
            } else {
                cout << ">>> Face detection is: " << s << "\n";
                faces_flag = false;
            }
        } else {
            cout << ">>> Face detection is: OFF" << "\n";
            faces_flag = false;
        }

        if (vm.count("saliency")) {
            const string& s = vm["saliency"].as<string>();
            if(s=="ON") {
                cout << ">>> Saliency detection is: " << s << "\n";
                saliency_flag = true;
            } else {
                 cout << ">>> Saliency detection is: " << s << "\n";
                 saliency_flag = false;
            }
         } else {
            cout << ">>> Saliency detection is: OFF" << "\n";
            saliency_flag = false;
        }

        if (vm.count("viz")) {
            const string& s = vm["viz"].as<string>();
            if(s=="ON") {
                cout << ">>> Visualization is: " << s << "\n";
                viz_flag = true;
            } else {
                 cout << ">>> Visualization is: " << s << "\n";
                 viz_flag = false;
            }
         } else {
            cout << ">>> Visualization is: OFF" << "\n";
            viz_flag = false;
        }

        if (vm.count("fit")) {
            const string& s = vm["fit"].as<string>();
            if(s=="ON") {
                cout << ">>> Fitting is: " << s << "\n";
                fit_flag = true;
            } else {
                 cout << ">>> Fitting is: " << s << "\n";
                 fit_flag = false;
            }
         } else {
            cout << ">>> Fitting is: OFF" << "\n";
            fit_flag = false;
        }

        if (vm.count("timing")) {
            const string& s = vm["timing"].as<string>();
            if(s=="ON") {
                cout << ">>> Timing is: " << s << "\n";
                timing_flag = true;
            } else {
                 cout << ">>> Timing is: " << s << "\n";
                 timing_flag = false;
            }
         } else {
            cout << ">>> Timing is: OFF" << "\n";
            timing_flag = false;
        }

    } catch(std::exception& e) { cout << e.what() << "\n"; }

    // ROS
    ros::init(argc, (char **) argv, "robotgazetools");

    Size imSize(320,240);

    // CV Set Capture Device
    VideoCapture capture;
\
    int usingCamera = NMPTUtils::getVideoCaptureFromCommandLineArgs(capture, argc, (const char**)argv);

    if (!usingCamera--) {
        cout << "Sorry no static video file support for now" << endl;
        exit(1);
    }

    cv::Mat frame;
    cv::Mat frame_s;
    cv::Mat frame_f;

    if (usingCamera) {
        capture.set(CV_CAP_PROP_FRAME_WIDTH, imSize.width);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, imSize.height);
        capture.set(CV_CAP_PROP_FPS, 30);
    }

    // DLIB
    Faces fac;
    if(faces_flag) {
        fac.setPath(dlib_path, viz_flag, fit_flag);
    }

    // NMPT
    Saliency sal;
    if(saliency_flag){
        sal.setup(usingCamera, viz_flag);
    }

    while (cv::waitKey(1) <= 0) {

        boost::posix_time::ptime init = boost::posix_time::microsec_clock::local_time();

        capture >> frame;

        if(timing_flag) {
            boost::posix_time::ptime c = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration cdiff = c - init;
            cout << "[Capture] Time Consumption: " << cdiff.total_milliseconds() << " ms" << std::endl;
        }

        std_msgs::Header h;
        h.stamp = ros::Time::now();
        h.frame_id = "1";

        if (faces_flag) {
            frame_f = frame.clone();
            fac.getFaces(frame_f, h);
        }

        if (saliency_flag) {
            frame_s = frame.clone();
            sal.getSaliency(frame_s, h);
        }

        // ROS Spinner (send messages trigger loop)
        ros::spinOnce();

        if(timing_flag) {
            boost::posix_time::ptime a = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration adiff = a - init;
            cout << "[Main Loop] Time Consumption: " << adiff.total_milliseconds() << " ms" << std::endl;
        }
    }
}
