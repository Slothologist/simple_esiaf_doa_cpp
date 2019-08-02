//============================================================================
// Name        : sound-source-loc.cpp
// Author      : Frederic Pesquet (fpesquet at gmail dot com)
// Version     :
// Copyright   :
// Description :
//   Computes the direction of the source of the sound it hears.
//   It uses 2 microphones, and compute the time of arrival difference of sound
//   between them to estimate the sound source localization.
//
// Edited and Extended by Florian Lier [flier AT techfak.uni-bielefeld dot de]
//
//============================================================================

// esiaf include
#include <esiaf_ros/esiaf_ros.h>

// boost config read imports
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

// ROS
#include "ros/ros.h"

// STD
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <limits.h>
#include <string>

// DEFINES
#define SAMPLE_TYPE int16_t
#define SAMPLE_TYPE_ALSA SND_PCM_FORMAT_S16_LE

using namespace std;

boost::function<void(const std::vector<int8_t> &, const esiaf_ros::RecordingTimeStamps &)> simple_esiaf_callback;
void esiaf_handler(const std::vector<int8_t> &signal, const esiaf_ros::RecordingTimeStamps & timeStamps){ simple_esiaf_callback(signal, timeStamps); };


/**
 * An utility class to compute the running average of a signal
 */

class RunningAverage {
    int _nbValuesForAverage;
    int _nbValues;
    float _mean;

public:
    RunningAverage(int nbValuesForAverage) {
        _nbValuesForAverage = nbValuesForAverage;
        _mean = 0;
        _nbValues = 0;
    }

    void newValue(SAMPLE_TYPE v) {
        if (_nbValues < _nbValuesForAverage)
            _nbValues++;
        _mean = ((_mean * (_nbValues - 1)) + v) / (float)_nbValues;
    }

    SAMPLE_TYPE getMean() {
        return (SAMPLE_TYPE) _mean;
    }
};

/**
 * This class computes the direction of the source of the sound it hears.
 *
 * It uses 2 microphones, and compute the time of arrival difference of sound
 * between them to estimate the sound source localization.
 *
 */

class SoundSourceLoc {

    /**
     * Max time shift between right and left mic in number of samples. <br/>
     * This typically depends on the sample rate and the distance between
     * microphones. <br/>
     * You can either compute this with clever formulas involving sound speed
     * and microphones distance, or just try and put the max value you get with
     * extreme loc of sound. Guess what I did :-)
     */
    int _nbSamplesMaxDiff;

    /**
     * Take a point for sound loc is level > 110% of mean level. <br/>
     * This allows to compute sound loc only for "meaningful" sounds, not
     * background noise.
     */
    float _minLevelFactorForValidLoc = 1.10f;

    /**
    * Default Azimuth angle
    */
    float _defaultElevationLevel = 0.0f;

    /**
     * sound speed in meters per seconds
     */
    static constexpr float _soundSpeed = 344;

    /**
     * sound sampling rate in Hz
     */
    unsigned int _soundSamplingRate;

    /**
     * Distance between microphones in meters
     */
    float _distanceBetweenMicrophones = 0.5f;

    /** An utility to compute the running average of sound power */
    RunningAverage* _averageSoundLevel;


public:
    SoundSourceLoc(float distance_mics, float audio_activation_level) {


        _averageSoundLevel = new RunningAverage(50);
        _soundSamplingRate = 44100;
        _distanceBetweenMicrophones = distance_mics;
        _minLevelFactorForValidLoc = audio_activation_level;
        _nbSamplesMaxDiff = (_distanceBetweenMicrophones/_soundSpeed)*_soundSamplingRate+1;
      
        cout << "Audio Activation Level --> " << _minLevelFactorForValidLoc << endl;
        cout << "Audio Sampling Rate --> " << _soundSamplingRate << endl;
        cout << "Microphone Distance (metres) --> " << _distanceBetweenMicrophones << endl;
        cout << "Max Sample Diff --> " << _nbSamplesMaxDiff << endl;
        cout << "Default Azimuth Level --> " << _defaultElevationLevel << endl;

    }

    /** Clean exit */
    ~SoundSourceLoc() {
        delete _averageSoundLevel;
    }

    /**
     * This is the core of the sound source localization: it takes the
     * right/left sampled sounds, and compute their differences while delaying
     * one channel more and more.<br/>
     * => the delay for which the difference is minimal is the real delay
     * between the right/left sounds, from which we can deduce the sound source
     * localization
     */
    float processNextSoundBlock(std::vector<SAMPLE_TYPE> _leftBuffer, std::vector<SAMPLE_TYPE> _rightBuffer) {
        size_t _bufferSize = _leftBuffer.size();

        // compute the sound level (i.e. "loudness" of the sound):
        SAMPLE_TYPE level = computeLevel(_rightBuffer, _leftBuffer);
        // update the average sound level with this new measure:
        _averageSoundLevel->newValue(level);
        // relative sound level of this sample compared to average:
        float relativeLevel = (float) level
                / (float) _averageSoundLevel->getMean();
        //cout << "level " << level << ", relative  " << relativeLevel << endl;

        int minDiff = INT_MAX;
        int minDiffTime = -1;
        // 'slide' time to find minimum of right/left sound differences
        for (int t = -_nbSamplesMaxDiff; t < _nbSamplesMaxDiff; t++) {
            // compute sum of differences as the cross-correlation-like measure:
            int diff = 0;
            for (int i = _nbSamplesMaxDiff;
                    i < _bufferSize - _nbSamplesMaxDiff - 1; i++) {
                diff += abs(_leftBuffer[i] - _rightBuffer[i + t]);
            }
            if (diff < minDiff) {
                minDiff = diff;
                minDiffTime = t;
            }
        }

        // if sound is loud enough, and not an extreme (=usually false
        // measure), then draw it:
        if ((relativeLevel > _minLevelFactorForValidLoc)
                && (minDiffTime > -_nbSamplesMaxDiff)
                && (minDiffTime < _nbSamplesMaxDiff)) {
            // computation of angle depending on diff time, sampling rates,
            // and geometry (thanks Mathieu from Pobot :-) ):
            float angle =
                    -(float) asin(
                            (minDiffTime * _soundSpeed)
                                    / (_soundSamplingRate
                                            * _distanceBetweenMicrophones));

            // cout << angle << " <--- " << relativeLevel << endl;
            // Set Positive avalues only 0 == left, 180 == right
            float degree = (angle*180/M_PI)+90.0f;

            if ( std::isnan(degree) == false ) {
                return degree;
            }
        }
        return 0.0;
    }

private:
    /**
     * Compute average sound level (i.e. power) for left/right channels.
     *
     * Notice we could probably do the computation on some samples only (for
     * example one over 4 samples) without loosing much accuracy here. This
     * would reduce computation time.
     * Also, as we are only interested in relative evolution, we could
     * simplify and avoid the multiplications by just taking the mean of
     * absolute values?
     */
    SAMPLE_TYPE computeLevel(std::vector<SAMPLE_TYPE> right, std::vector<SAMPLE_TYPE> left) {
        size_t _bufferSize = right.size();
        float level = 0;
        for (int i = 0; i < _bufferSize; i++) {
            float s = (left[i] + right[i]) / 2;
            level += (s * s);
        }
        level /= _bufferSize;
        level = sqrt(level);
        return (SAMPLE_TYPE) level;
    }

};

int main(int argc, char *argv[]) {

    std::string config_file = argv[1];

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_file, pt);

    // ros initialisation
    ros::init(argc, argv, pt.get<std::string>("node_name"));
    ros::NodeHandle n;

    // some parameters for esiaf
    std::string input_topic = pt.get<std::string>("input_topic");
    std::string output_topic = pt.get<std::string>("output_topic");

    float mic_x_distance = pt.get<float>("mic_x_distance");
    float mic_y_distance = pt.get<float>("mic_y_distance");

    float audio_activation_level = pt.get<float>("audio_activation_level");

    int first_x_channel = pt.get<int>("first_x_channel");
    int second_x_channel = pt.get<int>("second_x_channel");
    int first_y_channel = pt.get<int>("first_y_channel");
    int second_y_channel = pt.get<int>("second_y_channel");

    SoundSourceLoc soundLocX(mic_x_distance, audio_activation_level);
    SoundSourceLoc soundLocY(mic_y_distance, audio_activation_level);

    // prepare esiaf

    ROS_INFO("starting esiaf initialisation...");

    // initialise esiaf
    esiaf_ros::Esiaf_Handler handler(&n, esiaf_ros::NodeDesignation::SSL);
    ROS_INFO("creating esiaf output topic...");

    //create format for output topic
    esiaf_ros::EsiafAudioTopicInfo topicInfo;

    esiaf_ros::EsiafAudioFormat allowedFormat;
    allowedFormat.rate = esiaf_ros::Rate::RATE_44100;
    allowedFormat.channels = 4;
    allowedFormat.bitrate = esiaf_ros::Bitrate::BIT_INT_16_SIGNED;
    allowedFormat.endian = esiaf_ros::Endian::LittleEndian;

    topicInfo.allowedFormat = allowedFormat;
    topicInfo.topic = output_topic;

    // notify esiaf about the output topic
    ROS_INFO("adding esiaf output topic...");
    handler.add_output_topic(topicInfo);


    // notify esiaf about the output topic
    ROS_INFO("adding input topic....");


    // here we add a method to transfer incoming audio to the audio player
    simple_esiaf_callback = [&](const std::vector<int8_t> &signal,
                                const esiaf_ros::RecordingTimeStamps &timeStamps) {

        const int8_t* buf8 = signal.data();
        int16_t* buf16 = (int16_t*) buf8;

        //float degreeX = soundLocX.processNextSoundBlock( );
        //float degreeY = soundLocY.processNextSoundBlock( );

    };

    topicInfo.topic = input_topic;
    handler.add_input_topic(topicInfo, esiaf_handler);

    // start esiaf
    ROS_INFO("starting esiaf...");
    handler.start_esiaf();




    exit(0);

}
