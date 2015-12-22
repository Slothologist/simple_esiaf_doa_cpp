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

#include <iostream>
#include <sstream>
#include <thread>

// Boost
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/tokenizer.hpp>
#include <boost/token_functions.hpp>

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
#include <opencv2/highgui/highgui.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>

using namespace std;
using namespace cv;
using namespace boost;
using namespace boost::program_options;

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
    }
}


void Faces::getFaces(cv::Mat _img, std_msgs::Header header)
{
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
    people_msgs::People people_msg;
    people_msgs::Person person_msg;
    for(unsigned long i = 0; i < faces.size(); ++i) {
        // cout << "left: " << faces[i].left() << " top: " << faces[i].top() << endl;
        // cout << "right: " << faces[i].right() << " bottom: " << faces[i].bottom() << endl;
        person_msg.name = "unkown";
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
        if(fit) {
            win.clear_overlay();
        }
        win.set_image(cimg);
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
      // DLIB
      BlockTimer bt;
      FastSalience salTracker;
      cv::Mat viz, sal;
      int usingCamera;
      vector<double> lqrpt{2,.5};
      LQRPointTracker salientSpot{2};
      // ROS
      ros::NodeHandle n;
      ros::Publisher pub_s;
      // SELF
      bool vizu;

};

Saliency::Saliency() {
    // pub_s = n.advertise<sensor_msgs::RegionOfInterest>("robotgazetools/saliency", 10);
    pub_s = n.advertise<geometry_msgs::PointStamped>("robotgazetools/saliency", 10);

}

Saliency::~Saliency(){}

void Saliency::setup(int camera, bool _vis) {
    cout << ">>> Setting up Saliency..." << endl;
    usingCamera = camera;
    bt.blockRestart(1);
    salientSpot.setTrackerTarget(lqrpt);
    vizu = _vis;
    cout << ">>> Done!" << endl;
}

void Saliency::getSaliency(cv::Mat im, std_msgs::Header header)
{
    double saltime, tottime;

    viz.create(im.rows, im.cols*2, CV_32FC3);

    bt.blockRestart(0);
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

    vizRect = viz(Rect(im.cols,0,im.cols, im.rows));
    cvtColor(sal, vizRect, CV_GRAY2BGR);
    if (usingCamera) flip(viz, viz, 1);

    tottime = bt.getCurrTime(1);
    bt.blockRestart(1);

    stringstream text;
    text << "FastSUN: " << (int)(saltime*1000) << " ms ; Total: " << (int)(tottime*1000) << " ms.";

    putText(viz, text.str(), Point(20,20), FONT_HERSHEY_SIMPLEX, .33, Scalar(255,0,255));\

    // cout << "Most Salient Point: X " << lqrpt[0]*sal.cols << " Y " << lqrpt[1]*sal.rows << endl;

    // ROI does not feature a HEADER, need to look into this!
    // sensor_msgs::RegionOfInterest roi_msg;
    // roi_msg.x_offset = lqrpt[0]*sal.cols;
    // roi_msg.y_offset = lqrpt[1]*sal.cols;
    // roi_msg.height = 1;
    // roi_msg.width = 1;
    // pub_s.publish(roi_msg);

    geometry_msgs::PointStamped ps;
    geometry_msgs::Point p;
    double mid_x = lqrpt[0]*sal.cols;
    double mid_y = lqrpt[1]*sal.rows;
    p.x = mid_x;
    p.y = mid_y;
    p.z = lqrpt[0]*sal.cols;
    ps.point = p;
    ps.header = header;
    pub_s.publish(ps);

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

        options_description all("Allowed options");
        all.add(general).add(dlib).add(saliency).add(faces).add(viz).add(fit);

        options_description visible("Allowed options");
        visible.add(general).add(dlib).add(saliency).add(faces).add(viz).add(fit);

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
                    cout << ">>> DLIB pose model path is: " << s << "\n";
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

    } catch(std::exception& e) { cout << e.what() << "\n"; }

    Size imSize(320,240);

    // CV Set Capture Device
    VideoCapture capture;
\
    int usingCamera = NMPTUtils::getVideoCaptureFromCommandLineArgs(capture, argc, (const char**)argv);

    if (!usingCamera--) {
        return 0;
    }

    cv::Mat fim, fim2, sim, sim2;

    if (usingCamera) {
        capture.set(CV_CAP_PROP_FRAME_WIDTH, imSize.width);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, imSize.height);
        capture.set(CV_CAP_PROP_FPS, 30);
    }

    // ROS
    ros::init(argc, (char **) argv, "robotgazetools");

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

    while (waitKey(1) <= 0) {

        capture >> fim2;
        capture >> sim2;

        std_msgs::Header h;
        h.stamp = ros::Time::now();
        h.frame_id = "1";

        if(usingCamera) {
            fim = fim2;
            sim = sim2;

        } else {
            double ratio = imSize.width * 1. / fim2.cols;
            resize(fim2, fim, Size(0,0), ratio, ratio, INTER_NEAREST);
            resize(sim2, sim, Size(0,0), ratio, ratio, INTER_NEAREST);
        }

        if (saliency_flag) {
            sal.getSaliency(sim, h);
        }

        if (faces_flag) {
            fac.getFaces(fim, h);
        }

        // ROS Spinner (send messages trigger loop)
        ros::spinOnce();
    }
}
