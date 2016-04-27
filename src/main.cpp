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
#include "Faces.h"
#include "Grabber.h"
#include "Saliency.h"
#include "Grabber_ROS.h"
#include "Grabber_RSB.h"
// #include "Grabber_XIMEA.h"


using namespace std;
using namespace cv;
using namespace boost::program_options;

int main(int argc, char *const argv[]) {

    // Global config
    string dlib_path = "None";
    string ros_input_scope = "/usb_cam/image_raw";
    string rsb_input_scope = "/usb_cam/image_raw";
    string ros_topic = "robotgazetools";
    string rsb_port = "5556";
    string rsb_host = "localhost";
    bool faces_flag = false;
    bool viz_flag = false;
    bool saliency_flag = false;
    bool fit_flag = false;
    bool timing_flag = false;
    bool is_native = true;
    bool is_ros = false;
    bool is_rsb = false;
    bool is_spread = false;
    int rate = 30;
    int width = 320;
    int height = 240;
    int throttle_hard = 0;
    double sal_s = 1.0;

    // Programm options
    try {

        options_description general("general options");
        general.add_options()
                ("help", "Show this help")
                ("version", "0.1");

        options_description dlib("dlib options");
        dlib.add_options()
                ("dlib", value<string>(), "the dlib pose model path");

        options_description saliency("saliency detection options");
        saliency.add_options()
                ("saliency", value<string>(), "detect most salient point ON|OFF");

        options_description faces("face detection options");
        faces.add_options()
                ("faces", value<string>(), "detect faces ON|OFF");

        options_description viz("visualization options");
        viz.add_options()
                ("viz", value<string>(), "visulatization ON|OFF");

        options_description fit("fitting options");
        fit.add_options()
                ("fit", value<string>(), "fitting ON|OFF");

        options_description timing("timing options");
        timing.add_options()
                ("timing", value<string>(), "timing ON|OFF");

        options_description framerate("framerate options");
        framerate.add_options()
                ("framerate", value<int>()->default_value(30), "framerate <value> (default is 30)");

        options_description throttle("throttle options");
        throttle.add_options()
                ("throttle", value<int>()->default_value(0), "throttle threads in nano seconds (default 0)");

        // options_description ximea("ximea options");
        // ximea.add_options()
        //         ("ximea", value<string>(), "ximea ON|OFF");

        options_description rossource("rossource options");
        rossource.add_options()
                ("rossource", value<string>(), "rossource=$topic");

        options_description rsbsource("rsbsource options");
        rsbsource.add_options()
                ("rsbsource", value<string>(), "rsbsource=$topic");

        options_description rostopic("rostopic options");
        rostopic.add_options()
                ("rostopic", value<string>(), "rostopic=$topic");

        options_description iwidth("width options");
        iwidth.add_options()
                ("width", value<int>(), "width=NUMBER (default is 240 pixel)");

        options_description iheight("height options");
        iheight.add_options()
                ("height", value<int>(), "height=NUMBER (default is 320 pixel)");

        options_description rsbport("rsbport options");
        rsbport.add_options()
                ("rsbport", value<string>(), "rsbport=$topic");

        options_description rsbhost("rsbhost options");
        rsbhost.add_options()
                ("rsbhost", value<string>(), "rsbhost=$topic");

        options_description salsens("saliency sensitivity options");
        salsens.add_options()
                ("salsens", value<double>(), "salsens=NUMBER (default is 1.0)");

        options_description all("Allowed options");
        all.add(general).add(dlib).add(saliency)
                .add(faces).add(viz).add(fit)
                .add(timing).add(framerate)
                .add(rossource).add(rostopic)
                .add(rsbsource).add(iwidth)
                .add(iheight).add(rsbhost)
                .add(rsbport).add(throttle).add(salsens);

        options_description visible("Allowed options");
        visible.add(general).add(dlib).add(saliency)
                .add(faces).add(viz).add(fit)
                .add(timing).add(framerate)
                .add(rossource).add(rostopic)
                .add(rsbsource).add(iwidth)
                .add(iheight).add(rsbhost)
                .add(rsbport).add(throttle).add(salsens);

        variables_map vm;

        store(parse_command_line(argc, argv, all), vm);

        if (vm.count("help")) {
            cout << visible;
            return 0;
        }

        if (vm.count("faces")) {
            const string &s = vm["faces"].as<string>();
            if (s == "ON") {
                cout << ">>> Face detection is: " << s << "\n";
                faces_flag = true;
                if (vm.count("dlib")) {
                    const string &s = vm["dlib"].as<string>();
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
            const string &s = vm["saliency"].as<string>();
            if (s == "ON") {
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
            const string &s = vm["viz"].as<string>();
            if (s == "ON") {
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
            const string &s = vm["fit"].as<string>();
            if (s == "ON") {
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
            const string &s = vm["timing"].as<string>();
            if (s == "ON") {
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

        if (vm.count("rossource")) {
            const string &s = vm["rossource"].as<string>();
            ros_input_scope = s;
            // ximea_flag = false;
            is_native = false;
            is_ros = true;
            is_rsb = false;
            cout << ">>> ROS source is:" << s << "\n";
        } else {
            cout << ">>> ROS source: OFF" << "\n";
            is_ros = false;
        }

        if (vm.count("rostopic")) {
            const string &s = vm["rostopic"].as<string>();
            ros_topic = s;
            cout << ">>> ROStopic is:" << s << "\n";
        } else {
            cout << ">>> ROStopic is:" << ros_topic <<"\n";
        }

        if (vm.count("width")) {
            int w = vm["width"].as<int>();
            width = w;
            cout << ">>> Image width is: " << width << "\n";
        } else {
            cout << ">>> Image width is: " << width << "\n";
        }

        if (vm.count("height")) {
            int h = vm["height"].as<int>();
            height = h;
            cout << ">>> Image height is: " << height << "\n";
        } else {
            cout << ">>> Image width is: " << height << "\n";
        }

        if (vm.count("salsens")) {
            double s = vm["salsens"].as<double>();
            sal_s = s;
            cout << ">>> Salience sensitivity is: " << s << "\n";
        } else {
            cout << ">>> Saliency sensitivity: " << sal_s << "\n";
        }

        if (vm.count("throttle")) {
            int t = vm["throttle"].as<int>();
            throttle_hard = t;
            cout << ">>> Throttle is: " << throttle_hard << "\n";
        } else {
            cout << ">>> Throttle is: " << throttle_hard << "\n";
        }

        if (vm.count("rsbsource")) {
            const string &s = vm["rsbsource"].as<string>();
            rsb_input_scope = s;
            // ximea_flag = false;
            is_native = false;
            is_ros = false;
            is_rsb = true;
            cout << ">>> RSB source is: " << s << "\n";
        } else {
            cout << ">>> RSB source: OFF" << "\n";
            is_rsb = false;
        }

        if (vm.count("rsbhost")) {
            const string &s = vm["rsbhost"].as<string>();
            rsb_host = s;
            cout << ">>> RSB host is: " << s << "\n";
        } else {
            cout << ">>> RSB using Spread " << rsb_host << "\n";
            is_spread = true;
        }

        if (vm.count("rsbport")) {
            const string &s = vm["rsbport"].as<string>();
            rsb_port = s;
            cout << ">>> RSB port is: " << s << "\n";
        } else {
            cout << ">>> RSB using Spread " << rsb_host << "\n";
            is_spread = true;
        }

        /*
        if (vm.count("ximea")) {
            const string &s = vm["ximea"].as<string>();
            if (s == "ON") {
                cout << ">>> Using XIMEA: " << s << "\n";
                ximea_flag = true;
                is_native = false;
                is_ros = false;
            } else {
                cout << ">>> Using XIMEA: " << s << "\n";
                ximea_flag = false;
            }
        } else {
            cout << ">>> Using XIMEA: OFF" << "\n";
            ximea_flag = false;
        }
        */

    } catch (std::exception &e) { cout << e.what() << "\n"; }

    // ROS
    ros::init(argc, (char **) argv, "robotgazetools");

    /*
    if (ximea_flag) {

        // XIMEA Grabber
        Grabber_XIMEA grabber;
        grabber.setCapture(argc, (const char **) argv, rate, timing_flag, width, height);
        thread g_t(&Grabber_XIMEA::grabImage, &grabber);

        // DLIB
        Faces fac;
        if (faces_flag) {
            fac.setPathXimea(&grabber, dlib_path, viz_flag, fit_flag);
        }
        thread f_t(&Faces::getFaces, &fac, faces_flag, timing_flag);

        // NMPT
        Saliency sal;
        if (saliency_flag) {
            sal.setupXimea(&grabber, grabber.getCamera(), viz_flag);
        }
        thread s_t(&Saliency::getSaliency, &sal, saliency_flag, timing_flag);
        cout << ">>> GRABBER RUNNING in XIMEA MODE" << "\n";
        rosSpin();
    }
    */

    if (is_ros) {

        // ROS Grabber
        Grabber_ROS grabber(timing_flag, width, height, ros_input_scope);

        // DLIB
        Faces fac(ros_topic);
        if (faces_flag) {
            fac.setPathROS(&grabber, dlib_path, viz_flag, fit_flag);
        }
        thread f_t(&Faces::getFaces, &fac, faces_flag, timing_flag, throttle_hard);

        // NMPT
        Saliency sal(ros_topic);
        if (saliency_flag) {
            sal.setupROS(&grabber, grabber.getCamera(), viz_flag, sal_s);
        }
        thread s_t(&Saliency::getSaliency, &sal, saliency_flag, timing_flag, throttle_hard);

        cout << ">>> GRABBER RUNNING in ROS MODE" << "\n";
        ros::spin();
    }

    if (is_rsb) {

        // ROS Grabber
        Grabber_RSB grabber(timing_flag, width, height, rsb_input_scope, rsb_host, rsb_port, is_spread);

        // DLIB
        Faces fac(ros_topic);
        if (faces_flag) {
            fac.setPathRSB(&grabber, dlib_path, viz_flag, fit_flag);
        }
        thread f_t(&Faces::getFaces, &fac, faces_flag, timing_flag, throttle_hard);

        // NMPT
        Saliency sal(ros_topic);
        if (saliency_flag) {
            unsigned int c = 1;
            sal.setupRSB(&grabber, c, viz_flag, sal_s);
        }
        thread s_t(&Saliency::getSaliency, &sal, saliency_flag, timing_flag, throttle_hard);

        cout << ">>> GRABBER RUNNING in RSB MODE" << "\n";
        ros::spin();
    }

    if (is_native) {

        // STD Device Grabber
        Grabber grabber;
        grabber.setCapture(argc, (const char **) argv, rate, timing_flag, width, height);
        thread g_t(&Grabber::grabImage, &grabber);

        // DLIB
        Faces fac(ros_topic);
        if (faces_flag) {
            fac.setPath(&grabber, dlib_path, viz_flag, fit_flag);
        }
        thread f_t(&Faces::getFaces, &fac, faces_flag, timing_flag, throttle_hard);

        // NMPT
        Saliency sal(ros_topic);
        if (saliency_flag) {
            sal.setup(&grabber, grabber.getCamera(), viz_flag, sal_s);
        }
        thread s_t(&Saliency::getSaliency, &sal, saliency_flag, timing_flag, throttle_hard);

        cout << ">>> GRABBER RUNNING in NATIVE MODE" << "\n";
        ros::spin();
    }

}
