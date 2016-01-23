#pragma once

// SELF
#include "Grabber.h"
#include "Grabber_ROS.h"
#include "Grabber_RSB.h"

// NMPT
#include <nmpt/BlockTimer.h>
#include <nmpt/FastSalience.h>
#include <nmpt/LQRPointTracker.h>

class Saliency {
public:
    Saliency(std::string topic);
    ~Saliency();
    void getSaliency(bool saliency_flag, bool timing, int throttle);
    void setup(Grabber *grab, int camera, bool _vis, double _sal_sens);
    void setupROS(Grabber_ROS *grab, int camera, bool _vis, double _sal_sens);
    void setupRSB(Grabber_RSB *grab, int camera, bool _vis, double _sal_sens);
    // void setupXimea(Grabber_XIMEA *grab, int camera, bool _vis);
protected:
    // NMPT
    BlockTimer bt;
    cv::Mat viz, sal;
    int usingCamera;

    // ROS
    ros::NodeHandle n;
    ros::Publisher pub_s;

    // SELF
    bool vizu, is_ximea, is_ros, is_native, is_rsb;
    Grabber *grabber;
    Grabber_ROS *grabber_ros;
    Grabber_RSB *grabber_rsb;
    // Grabber_XIMEA *grabber_x;

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
    std::vector<double> lqrpt{2, .5};
    // Default: salientSpot{2};
    LQRPointTracker * salientSpot;
};
