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
// Edited and Extended by Florian Lier [flier AT techfak.uni-bielefeld.de]
//
//============================================================================

// ROS
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"

// STD
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <limits.h>
#include <alsa/asoundlib.h>

// DEFINES
#define SAMPLE_TYPE short
#define SAMPLE_TYPE_ALSA SND_PCM_FORMAT_S16_LE

using namespace std;

/* ROS Class to send Messages */

class ROSComm {

private:
    ros::NodeHandle n;
    ros::Publisher pub_s;
public:
    ROSComm() {}
    ~ROSComm() {}

    void init_ros(int argc, char *const argv[]){
        pub_s = n.advertise<geometry_msgs::PointStamped>(argv[2], 2);
    }

    void send_ssloc(float _angle, float _sound_level) {
        std_msgs::Header h;
        h.stamp = ros::Time::now();
        h.frame_id = "0";

        geometry_msgs::PointStamped ps;
        geometry_msgs::Point p;
        p.x = _angle;
        p.y = 0.0;
        p.z = _sound_level;
        ps.point = p;
        ps.header = h;

        pub_s.publish(ps);
    }
};

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
    static const int _nbSamplesMaxDiff = 48;

    /**
     * Buffer size on which we will try to locate sound. <br/>
     * This is a number of samples, and depends on sample rate, and speed of
     * sound loc change we want to detect. Lower values mean compute sound loc
     * often, but accuracy is quite low as we compute on a very small slice of
     * sound. <br/>
     * Empirically, I found that computing on long sounds is better, here 4096
     * samples at 44 KHz sampling rate means about one second of sound => we
     * reevaluate sound loc every second. <br/>
     * Notice that the larger the value, the most computation we do, as we time
     * shift on the whole buffer.
     */
    static const int _bufferSize = 4096;

    /**
     * Take a point for sound loc is level > 110% of mean level. <br/>
     * This allows to compute sound loc only for "meaningful" sounds, not
     * background noise.
     */
    float _minLevelFactorForValidLoc = 1.10f;

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

    /** ALSA sound input handle */
    snd_pcm_t* _capture_handle;

    /** sound samples input buffer */
    SAMPLE_TYPE _rightBuffer[_bufferSize];
    SAMPLE_TYPE _leftBuffer[_bufferSize];

    // ROS STUFF
    ROSComm* rs;

public:
    SoundSourceLoc(int argc, char* argv[]) {

        ros::init(argc, (char **) argv, "robotgazetoolsaudio");
        rs = new ROSComm();
        rs->init_ros(argc, argv);

        _averageSoundLevel = new RunningAverage(50);
        _soundSamplingRate = 44100;
        _distanceBetweenMicrophones = atof(argv[3]);
        _minLevelFactorForValidLoc = atof(argv[4]);

        // sampling: 2 chanels, 44 KHz, 16 bits.
        int err;
        snd_pcm_hw_params_t* hw_params;

        // ideally use "hw:0,0" for embedded, to limit processing. But check if card support our needs...
        const char* device = argv[1];

        if ((err = snd_pcm_open(&_capture_handle, device,
                SND_PCM_STREAM_CAPTURE, 0)) < 0) {
            fprintf(stderr, "cannot open audio device %s (%s)\n", device,
                    snd_strerror(err));
            exit(1);
        }

        if ((err = snd_pcm_hw_params_malloc(&hw_params)) < 0) {
            fprintf(stderr,
                    "cannot allocate hardware parameter structure (%s)\n",
                    snd_strerror(err));
            exit(1);
        }

        if ((err = snd_pcm_hw_params_any(_capture_handle, hw_params)) < 0) {
            fprintf(stderr,
                    "cannot initialize hardware parameter structure (%s)\n",
                    snd_strerror(err));
            exit(1);
        }

        if ((err = snd_pcm_hw_params_set_access(_capture_handle, hw_params,
                SND_PCM_ACCESS_RW_NONINTERLEAVED)) < 0) {
            fprintf(stderr, "cannot set access type (%s)\n", snd_strerror(err));
            exit(1);
        }

        if ((err = snd_pcm_hw_params_set_format(_capture_handle, hw_params,
                SAMPLE_TYPE_ALSA)) < 0) {
            fprintf(stderr, "cannot set sample format (%s)\n",
                    snd_strerror(err));
            exit(1);
        }

        if ((err = snd_pcm_hw_params_set_rate_near(_capture_handle, hw_params,
                &_soundSamplingRate, 0)) < 0) {
            fprintf(stderr, "cannot set sample rate (%s)\n", snd_strerror(err));
            exit(1);
        }

        if ((err = snd_pcm_hw_params_set_channels(_capture_handle, hw_params, 2))
                < 0) {
            fprintf(stderr, "cannot set channel count (%s)\n",
                    snd_strerror(err));
            exit(1);
        }

        if ((err = snd_pcm_hw_params(_capture_handle, hw_params)) < 0) {
            fprintf(stderr, "cannot set parameters (%s)\n", snd_strerror(err));
            exit(1);
        }

        snd_pcm_hw_params_free(hw_params);

        if ((err = snd_pcm_prepare(_capture_handle)) < 0) {
            fprintf(stderr, "cannot prepare audio interface for use (%s)\n",
                    snd_strerror(err));
            exit(1);
        }
    }

    /** Clean exit */
    ~SoundSourceLoc() {
        snd_pcm_close(_capture_handle);
        delete _averageSoundLevel;
    }

    /**
     * Main loop: read a buffer, compute sound source localization, iterate.
     */
    void run() {
        while (true) {
            processNextSoundBlock();
        }
    }

private:
    /**
     * This is the core of the sound source localization: it takes the
     * right/left sampled sounds, and compute their differences while delaying
     * one channel more and more.<br/>
     * => the delay for which the difference is minimal is the real delay
     * between the right/left sounds, from which we can deduce the sound source
     * localization
     */
    void processNextSoundBlock() {
        SAMPLE_TYPE* bufs[2];
        bufs[0] = _rightBuffer;
        bufs[1] = _leftBuffer;
        int err;
        if ((err = snd_pcm_readn(_capture_handle, (void**) bufs, _bufferSize))
                != _bufferSize) {
            fprintf(stderr, "read from audio interface failed (%s)\n",
                    snd_strerror(err));
            exit(1);
        }

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
            float degree = angle*180/M_PI;

            // Set Positive avalues only 0 == left, 180 == right
            if ( std::isnan(degree) == false ) {
                cout << (angle*180/M_PI)+90.0f << " <--- Degree " << relativeLevel << " <--- Relative Sound Level" << endl;
                rs->send_ssloc(degree, relativeLevel);
            }
        }
    }

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
    SAMPLE_TYPE computeLevel(SAMPLE_TYPE right[], SAMPLE_TYPE left[]) {
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

    if (argc < 4) {
        cout << "You need to provide the following arguments: DEVICENAME OUTTOPIC DISTANCE LEVEL";
    }

    SoundSourceLoc soundLoc(argc, argv);
    soundLoc.run();
    exit(0);

}
