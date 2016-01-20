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


Grabber_RSB::Grabber_RSB(bool _timing, int _width, int _height, std::string _scope, std::string _host, bool socket_input) {
    timing = _timing;
    width = _width;
    height = _height;
    imageQueue = ImageQueuePtr(new ImageQueue(1));
    imageHandler = ImageHandlerPtr(new ImageHandler(imageQueue));
    rsb::converter::Converter<std::string>::Ptr image_c(new rst::converters::opencv::IplImageConverter());
    rsb::converter::converterRepository<string>()->registerConverter(image_c);

    rsb::Factory &factory = rsb::getFactory();

    // Create listener
    rsb::ParticipantConfig listenerConfig = factory.getDefaultParticipantConfig();
    if (socket_input) {
      set<rsb::ParticipantConfig::Transport> enabledTransports = listenerConfig.getTransports();
      for (set<rsb::ParticipantConfig::Transport>::const_iterator transportIt =
              enabledTransports.begin(); transportIt != enabledTransports.end();
              ++transportIt) {
          listenerConfig.mutableTransport(transportIt->getName()).setEnabled(false);
      }
      listenerConfig.mutableTransport("socket").setEnabled(true);
      listenerConfig.mutableTransport("socket").mutableOptions().set<string>("host", _host);
      listenerConfig.mutableTransport("socket").mutableOptions().set<string>("port", "5556");
    }

    imageListener = factory.createListener(rsb::Scope(_scope), listenerConfig);
}

Grabber_RSB::~Grabber_RSB() {}

void Grabber_RSB::grab() {
    rsb::EventPtr imageEvent = imageQueue->pop();
    if (imageEvent->getType() == rsc::runtime::typeName<IplImage>()) {
        ros::Time now = ros::Time::now();
        timestamp = now;
        boost::shared_ptr<IplImage> image = boost::static_pointer_cast<IplImage>(imageEvent->getData());
        imageMetaData = imageEvent->getMetaData();
        source_frame = image.get();
        mtx.lock();
        cv::Size size(width, height);
        if (source_frame.rows > 0 && source_frame.cols > 0 && source_frame.rows != width && source_frame.cols != height) {
             cv::resize(source_frame, output_frame, size);
        } else {
            output_frame = source_frame;
        }
        mtx.unlock();
    }
}

void Grabber_RSB::getImage(ros::Time *target_timestamp, cv::Mat *image) {
    mtx.lock();
    *image = output_frame;
    if (target_timestamp != NULL) {
        *target_timestamp = timestamp;
    }
    mtx.unlock();
}
