/*
 *  simple_robot_gaze_tools
 *
 *  Created by Nicholas Butko on 11/15/10.
 *  Used and extended by Florian Lier on December 2015
 *  Copyright 2010 UC San Diego. All rights reserved.
 *  Copyright 2015 Bielefeld University. All rights reserved.
 *
 */

#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <thread>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/tokenizer.hpp>
#include <boost/token_functions.hpp>

#include <nmpt/BlockTimer.h>
#include <nmpt/FastSalience.h>
#include <nmpt/LQRPointTracker.h>
#include <nmpt/NMPTUtils.h>

#include "ros/ros.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "people_msgs/People.h"

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

class Faces
{
    public:
      Faces();
      ~Faces();
      void getFaces(cv::Mat img);
      void setPath(String path);

    protected:
      // DLIB
      dlib::frontal_face_detector detector;
      dlib::shape_predictor pose_model;
      dlib::image_window win;
};

Faces::Faces(){}
Faces::~Faces(){}

void Faces::setPath(String path)
{
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


void Faces::getFaces(cv::Mat _img)
{
    dlib::cv_image<dlib::bgr_pixel> cimg(_img);
    // Detect faces
    std::vector<dlib::rectangle> faces = detector(cimg);
    // Find the pose of each face.
    std::vector<dlib::full_object_detection> shapes;
    for (unsigned long i = 0; i < faces.size(); ++i) {
        shapes.push_back(pose_model(cimg, faces[i]));
    }
    // Display it all on the screen
    win.clear_overlay();
    win.set_image(cimg);
    win.add_overlay(render_face_detections(shapes));
}


int main (int argc, char * const argv[])
{
    String dlib_path = "None";
    bool faces_flag = false;
    bool saliency_flag = false;

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

        options_description all("Allowed options");
        all.add(general).add(dlib).add(saliency).add(faces);

        options_description visible("Allowed options");
        visible.add(general).add(dlib).add(saliency).add(faces);

        variables_map vm;

        store(parse_command_line(argc, argv, all), vm);

        if (vm.count("help"))
        {
            cout << visible;
            return 0;
        }

        if (vm.count("faces")) {
            cout << ">>> face detection is ON" << "\n";
            faces_flag = true;
            if (vm.count("dlib")) {
                const string& s = vm["dlib"].as<string>();
                dlib_path = s;
                cout << "    dlib pose model path is: " << s << "\n";
            } else {
                cout << ">>> ERROR: dlib pose model path NOT set \n";
                faces_flag = false;
                return 0;
            }
        } else {
            cout << ">>> face detection is OFF" << "\n";
            faces_flag = false;
        }

        if (vm.count("saliency")) {
            cout << ">>> saliency detection is ON" << "\n";
            saliency_flag = true;
        } else {
            cout << ">>> saliency detection is OFF" << "\n";
            saliency_flag = false;
        }

     } catch(std::exception& e) { cout << e.what() << "\n"; }

    Size imSize(320,240);
    BlockTimer bt;

    //CV set capture device
    VideoCapture capture;
\
    int usingCamera = NMPTUtils::getVideoCaptureFromCommandLineArgs(capture, argc, (const char**)argv);
    if (!usingCamera--) return 0;

    FastSalience salTracker;

    LQRPointTracker salientSpot(2);
    vector<double> lqrpt(2,.5);
    salientSpot.setTrackerTarget(lqrpt);

    //CV set capture to desired width/height
    if (usingCamera) {
        capture.set(CV_CAP_PROP_FRAME_WIDTH, imSize.width);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, imSize.height);
    }

    bt.blockRestart(1);
    cv::Mat im, im2, viz, sal;

    // ROS
    ros::init(argc, (char **) argv, "gazetools");
    ros::NodeHandle n;
    ros::Publisher pub_s = n.advertise<sensor_msgs::RegionOfInterest>("gazetools/saliency", 10);
    ros::Publisher pub_f = n.advertise<people_msgs::People>("gazetools/faces", 10);

    // DLIB
    Faces f;
    if(faces_flag) {
        f.setPath(dlib_path);
    }

    // thread c_thread;

    while (waitKey(5) <= 0) {

            double saltime, tottime;
            capture >> im2;
            if (usingCamera) {
                im = im2;
            } else {
                double ratio = imSize.width * 1. / im2.cols;
                resize(im2, im, Size(0,0), ratio, ratio, INTER_NEAREST);
            }

            /*if(faces_flag && c_thread.get_id() == thread::id()) {
                //f.getFaces(im);
                //thread t(bind(Faces::getFaces, f, im));
                c_thread = thread(bind(&Faces::getFaces, &f, im));
            }*/

            f.getFaces(im);

            if (saliency_flag) {
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

                cout << "Most Salient Point: X " << lqrpt[0]*sal.cols << " Y " << lqrpt[1]*sal.cols << endl;

                sensor_msgs::RegionOfInterest roi_msg;
                roi_msg.x_offset = lqrpt[0]*sal.cols;
                roi_msg.y_offset = lqrpt[1]*sal.cols;
                roi_msg.height = 1;
                roi_msg.width = 1;

                pub_s.publish(roi_msg);

                imshow("SRG-Tools || NMPT Salience || Press Q to Quit", viz);
            }

        // ROS Spinner (send messages)
        ros::spinOnce();
    }
}
