// SELF
#include "Saliency.h"

// ROS
#include "ros/ros.h"
#include "people_msgs/People.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"

// BOOST
#include "boost/date_time/posix_time/posix_time.hpp"

using namespace std;
using namespace cv;

Saliency::Saliency() {
    pub_s = n.advertise<geometry_msgs::PointStamped>("robotgazetools/saliency", 10);
}

Saliency::~Saliency(){}

void Saliency::setup(Grabber_XIMEA* grab, int camera, bool _vis) {
    cout << ">>> Setting up Saliency..." << endl;
    grabber = grab;
    usingCamera = camera;
    bt.blockRestart(1);
    salientSpot.setTrackerTarget(lqrpt);
    salTracker.setUseDoEFeatures(1);
    vizu = _vis;
    cout << ">>> Done!" << endl;
}

void Saliency::getSaliency(bool saliency_flag, bool timing)
{

    ros::Time old = ros::Time::now();

    while(cv::waitKey(1) <= 0) {

        if (!saliency_flag) {
            usleep(5000);
            return;
        }

        boost::posix_time::ptime init = boost::posix_time::microsec_clock::local_time();

        ros::Time frame_timestamp;
        cv::Mat im = grabber->getImage(&frame_timestamp);

        // If no image has been grabbed yet...wait.
        if (im.rows == 0 || im.cols == 0) {
            cout << "Saliency: waiting for next image to be grabbed..." << endl;
            continue;
        }

        std_msgs::Header h;
        h.stamp = frame_timestamp;
        if (h.stamp <= old){
            usleep(1000);
            continue;
        }
        h.frame_id = "1";
        old = h.stamp;

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
        ps.header = h;
        pub_s.publish(ps);

        tottime = bt.getCurrTime(1);

        // Stop Timer
        bt.blockRestart(1);

        stringstream text;
        text << "FastSUN: " << (int)(saltime*1000) << " ms ; Total: " << (int)(tottime*1000) << " ms.";

        putText(viz, text.str(), Point(20,20), FONT_HERSHEY_SIMPLEX, .33, Scalar(255,0,255));\

        if(vizu) {
            imshow("Simple Robot Gaze Tools || NMPT Salience || Press Q to Quit", viz);
        }

        if(timing) {
            boost::posix_time::ptime c = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration cdiff = c - init;
            cout << "[SALIENCY] Time Consumption: " << cdiff.total_milliseconds() << " ms" << std::endl;
        }
    }
}
