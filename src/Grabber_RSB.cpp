// SELF
#include "Grabber_RSB.h"

// RSC
#include <rsc/misc/langutils.h>

// RSB
#include <rsb/Listener.h>
#include <rsb/Factory.h>
#include <rsb/filter/ScopeFilter.h>
#include <rsb/converter/Converter.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/util/EventQueuePushHandler.h>
#include <rsb/Listener.h>
#include <rsb/Factory.h>
#include <rsb/MetaData.h>
#include <rsb/EventCollections.h>
#include <rsb/converter/EventsByScopeMapConverter.h>

// BOOST
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

// RST
#include <rst/vision/Faces.pb.h>
#include <rst/vision/Face.pb.h>
#include <rst/math/Vec2DInt.pb.h>
#include <rst/geometry/BoundingBox.pb.h>
#include <rst/stochastics/MixtureOfGaussian1D.pb.h>
#include <rst/vision/HeadObjects.pb.h>
#include <rst/vision/HeadObject.pb.h>
#include <rst/converters/opencv/IplImageConverter.h>

// CV
#include <opencv/cv.h>
#include <opencv/highgui.h>


using namespace std;
using namespace rsc::logging;
using namespace rst::stochastics;
using namespace rst::vision;
using namespace rst::math;
using namespace rst::geometry;


Grabber_RSB::Grabber_RSB(bool _timing, int _width, int _height, std::string _scope, std::string _host, std::string _port, bool _is_spread) {

    timing = _timing;
    width = _width;
    height = _height;
    host = _host;
    port = _port;
    imageQueue = ImageQueuePtr(new ImageQueue(5));
    imageHandler = ImageHandlerPtr(new ImageHandler(imageQueue));

    rsb::Factory &factory = rsb::getFactory();

    try {
        rsb::converter::Converter<std::string>::Ptr image_c(new rst::converters::opencv::IplImageConverter());
        rsb::converter::converterRepository<string>()->registerConverter(image_c);
    } catch(...) {
        cout << ">> RSB IS WEIRD (converter already registered)" << endl;
    }

    rsb::ParticipantConfig listenerConfig = factory.getDefaultParticipantConfig();

    set<rsb::ParticipantConfig::Transport> enabledTransports = listenerConfig.getTransports();

    for (set<rsb::ParticipantConfig::Transport>::const_iterator transportIt =
         enabledTransports.begin(); transportIt != enabledTransports.end();
         ++transportIt) {
        listenerConfig.mutableTransport(transportIt->getName()).setEnabled(false);
    }

    if(!_is_spread) {
        listenerConfig.mutableTransport("socket").setEnabled(true);
        listenerConfig.mutableTransport("socket").mutableOptions().set<string>("host", host);
        listenerConfig.mutableTransport("socket").mutableOptions().set<string>("port", port);
        listenerConfig.mutableTransport("socket").mutableOptions().set<string>("server", "0");

        imageListener = factory.createListener(rsb::Scope(_scope), listenerConfig);
        imageListener->addHandler(imageHandler);
    } else {
        imageListener = factory.createListener(rsb::Scope(_scope));
        imageListener->addHandler(imageHandler);
    }
}

Grabber_RSB::~Grabber_RSB() {}

void Grabber_RSB::getImage(ros::Time *target_timestamp, cv::Mat *image) {
    rsb::EventPtr imageEvent = imageQueue->pop();
    mtx.lock();
    if (imageEvent->getType() == rsc::runtime::typeName<IplImage>()) {
        // TODO: Implement createTime to ROS::TIME
        ros::Time now = ros::Time::now();
        *target_timestamp = now;
        boost::shared_ptr<IplImage> newImage = boost::static_pointer_cast<IplImage>(imageEvent->getData());
        imageMetaData = imageEvent->getMetaData();
        if (newImage->width != width && newImage->height != height) {
            cv::Size size(width, height);
            cv::resize(cv::Mat(newImage.get(), false), *image, size);
        } else {
            (*image) = cv::Mat(newImage.get(), true);
        }
    }
    mtx.unlock();
}
