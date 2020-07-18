//
// Created by flk on 20-4-3.
//

#ifndef ORB_VINS_SLAM_VIEWER_H
#define ORB_VINS_SLAM_VIEWER_H

//#include "FrameDrawer.h"
//#include "MapDrawer.h"
#include "tracking.h"
#include "system.h"
#include "mapdrawer.h"
#include "frame.h"

#include <pangolin/pangolin.h>

#include <mutex>

namespace ovslam
{

    class Tracking;
    class FrameDrawer;
    class MapDrawer;
    class System;

    class Viewer
    {
    public:
        Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath);

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
        // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
        void Run();

        void RequestFinish();

        void RequestStop();

        bool isFinished();

        bool isStopped();

        void Release();

    private:

        bool Stop();

        System* mpSystem;
        FrameDrawer* mpFrameDrawer;
        MapDrawer* mpMapDrawer;
        Tracking* mpTracker;

        // 1/fps in ms
        double mT;
        float mImageWidth, mImageHeight;

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        std::mutex mMutexStop;

    };

}









#endif //ORB_VINS_SLAM_VIEWER_H
