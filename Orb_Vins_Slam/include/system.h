//
// Created by flk on 20-4-2.
//

#ifndef ORB_VINS_SLAM_SYSTEM_H
#define ORB_VINS_SLAM_SYSTEM_H

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <thread>
#include "map.h"
#include "tracking.h"
#include "viewer.h"
#include "framedrawer.h"
#include "frame.h"

using namespace std;

namespace ovslam
{

class Viewer;
class Tracking;
class MapDrawer;
class KeyFrameDatabase;

    class System
{
    public:

        // Input sensor
        enum eSensor{
            MONOCULAR=0,
            STEREO=1,
            RGBD=2
        };

        eSensor mSensor;

    //////////////////////////////////////////////////////////////////////////////////

    // Initialize the SLAM system
    System( const string &strSettingsFile, const string &strVocFile, const eSensor sensor,const bool bUseViewer);//  ×××构造函数，初始化一个SLAM系统

    void DeactivateLocalizationMode();

    void ActivateLocalizationMode();

    // Reset the system (clear map)
    void Reset();

    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    protected:

    private:

        // ORB vocabulary used for place recognition and feature matching.
        ORBVocabulary* mpVocabulary;

        // KeyFrame database for place recognition (relocalization and loop detection).
        KeyFrameDatabase* mpKeyFrameDatabase;   //   KeyFrameDatabase 是一个类

        // Map structure that stores the pointers to all KeyFrames and MapPoints.
        Map* mpMap;

        // Tracker. It receives a frame and computes the associated camera pose.
        // It also decides when to insert a new keyframe, create some new MapPoints and
        // performs relocalization if tracking fails.
        Tracking* mpTracker;

        // Reset flag
        std::mutex mMutexReset;
        bool mbReset;

        // Change mode flags
        std::mutex mMutexMode;
        bool mbActivateLocalizationMode;
        bool mbDeactivateLocalizationMode;

        // The viewer draws the map and the current camera pose. It uses Pangolin.
        Viewer* mpViewer;

        // System threads: Local Mapping, Loop Closing, Viewer.
        // The Tracking thread "lives" in the main execution thread that creates the System object.
        std::thread* mptLocalMapping;
        std::thread* mptLoopClosing;
        std::thread* mptViewer;

        FrameDrawer* mpFrameDrawer;
        MapDrawer* mpMapDrawer;

};
}

#endif //ORB_VINS_SLAM_SYSTEM_H
