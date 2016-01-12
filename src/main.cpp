// STD
#include <iostream>
#include <sstream>
#include <string>

// CV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// BOOST
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

// THREADING
#include <thread>
#include <mutex>

// SELF
#include "Saliency.h"
#include "Grabber.h"
#include "Faces.h"


using namespace std;
using namespace cv;
using namespace boost::program_options;

int main (int argc, char * const argv[])
{
    // Global toogle/config
    string dlib_path = "None";
    bool faces_flag = false;
    bool viz_flag = false;
    bool saliency_flag = false;
    bool fit_flag = false;
    bool timing_flag = false;
    int rate = 30;

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

        options_description framerate("framerate options");
        timing.add_options()
            ("frameratfrontal_face_detectore", value<int>()->default_value(30), "framerate <value> (default is 30)")
            ;

        options_description all("Allowed options");
        all.add(general).add(dlib).add(saliency).add(faces).add(viz).add(fit).add(timing).add(framerate);

        options_description visible("Allowed options");
        visible.add(general).add(dlib).add(saliency).add(faces).add(viz).add(fit).add(timing).add(framerate);

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
            }class Saliency
            {
                public:
                  Saliency();
                  ~Saliency();
                  void getSaliency(bool saliency_flag, bool timing);
                  void setup(Grabber* grab, int camera, bool _vis);
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
                  Grabber* grabber;
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

        if (vm.count("framerate")) {
            int f = vm["framerate"].as<int>();
            rate = f;
            cout << ">>> Framerate is: " << rate << "\n";
        } else {
            cout << ">>> Framerate is: " << rate << "\n";
        }

    } catch(std::exception& e) { cout << e.what() << "\n"; }

    // ROS
    ros::init(argc, (char **) argv, "robotgazetools");

    // Workaround for 2 Threads using imshow()
    namedWindow("Simple Robot Gaze Tools || NMPT Salience || Press Q to Quit");

    // Grabber Thread
    Grabber grab;
    grab.setCapture(argc, (const char**) argv, rate);
    thread g_t(&Grabber::grabImage, &grab);

    // DLIB
    Faces fac;
    if(faces_flag) {
        fac.setPath(&grab, dlib_path, viz_flag, fit_flag);
    }
    thread f_t(&Faces::getFaces, &fac, faces_flag, timing_flag);

    // NMPT
    Saliency sal;
    if(saliency_flag){
        sal.setup(&grab, grab.getCamera(), viz_flag);
    }
    thread s_t(&Saliency::getSaliency, &sal, saliency_flag, timing_flag);

    ros::Rate r(100);

    while(cv::waitKey(1) <= 0) {
        ros::spinOnce();
        r.sleep();
    }
}
