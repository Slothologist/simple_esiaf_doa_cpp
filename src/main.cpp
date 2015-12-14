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

int main (int argc, char * const argv[])
{
    Size imSize(320,240);
    BlockTimer bt;

    /* Open capture */
    VideoCapture capture;
    int usingCamera = NMPTUtils::getVideoCaptureFromCommandLineArgs(capture, argc, (const char**) argv);
    if (!usingCamera--) return 0;

    FastSalience salTracker;

    LQRPointTracker salientSpot(2);
    vector<double> lqrpt(2,.5);
    salientSpot.setTrackerTarget(lqrpt);

    /* Set capture to desired width/height */
    if (usingCamera) {
        capture.set(CV_CAP_PROP_FRAME_WIDTH, imSize.width);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, imSize.height);
    }
    bt.blockRestart(1);
    cv::Mat im, im2, viz, sal, temp;

    // ROS
    ros::init(argc, (char **) argv, "gazetools");
    ros::NodeHandle n;

    ros::Publisher pub_s = n.advertise<sensor_msgs::RegionOfInterest>("gazetools/saliency", 10);
    ros::Publisher pub_f = n.advertise<people_msgs::People>("gazetools/faces", 10);

    // DLIB
    // Load face detection and pose estimation models.
    dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
    dlib::shape_predictor pose_model;
    dlib::deserialize("shape_predictor_68_face_landmarks.dat") >> pose_model;
    dlib::image_window win;

    while (waitKey(5) <= 0) {
        try  {
            double saltime, tottime;
            capture >> im2;
            capture >> temp;
            // im2 >> temp;
            if (usingCamera) {
                im = im2;
            } else {
                double ratio = imSize.width * 1. / im2.cols;
                resize(im2, im, Size(0,0), ratio, ratio, INTER_NEAREST);
            }

            dlib::cv_image<dlib::bgr_pixel> cimg(temp);
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

            people_msgs::People people_msg;
            //people_msg.people.
            //roi_msg.x_offset = lqrpt[0]*sal.cols;
            //roi_msg.y_offset = lqrpt[1]*sal.cols;
            //roi_msg.height = 1;
            //roi_msg.width = 1;

            pub_s.publish(roi_msg);
            //pub_f.publish(roi_msg);
            ros::spinOnce();

            imshow("SRG-Tools || Salience", viz);

        }catch(dlib::serialization_error& e) {
            cout << "You need dlib's default face landmarking model file to run this example." << endl;
            cout << "You can get it from the following URL: " << endl;
            cout << "http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2" << endl;
            cout << endl << e.what() << endl;
        }
        catch(exception& e) {
            cout << e.what() << endl;
        }
    }
}
