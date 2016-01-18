#pragma once

// SELF
#include "Grabber.h"
#include "Grabber_ROS.h"

// NMPT
#include <nmpt/BlockTimer.h>
#include <nmpt/FastSalience.h>
#include <nmpt/LQRPointTracker.h>

class Saliency {
public:
    Saliency();
    ~Saliency();
    void getSaliency(bool saliency_flag, bool timing);
    void setup(Grabber *grab, int camera, bool _vis);
    // void setupXimea(Grabber_XIMEA *grab, int camera, bool _vis);
    void setupROS(Grabber_ROS *grab, int camera, bool _vis);
protected:
    // NMPT
    BlockTimer bt;
    cv::Mat viz, sal;
    int usingCamera;
    // ROS
    ros::NodeHandle n;
    ros::Publisher pub_s;
    // SELF
    bool vizu, is_ximea, is_ros, is_native;
    // Grabber_XIMEA *grabber_x;
    Grabber *grabber;
    Grabber_ROS *grabber_ros;
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
    // Deafault: salientSpot{2};
    LQRPointTracker salientSpot{2, 1.0, 0, .015};
};
