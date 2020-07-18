//
// Created by flk on 20-4-3.
//

#ifndef ORB_VINS_SLAM_MAPDRAWER_H
#define ORB_VINS_SLAM_MAPDRAWER_H
#include "map.h"
#include "mappoint.h"
#include "keyframe.h"
#include <pangolin/pangolin.h>
#include "frame.h"

#include <mutex>

namespace ovslam
{

    class MapDrawer
    {
    public:
        MapDrawer(Map* pMap, const string &strSettingPath);

        Map* mpMap;

        void DrawMapPoints();
        void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
        void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
        void SetCurrentCameraPose(const cv::Mat &Tcw);
        void SetReferenceKeyFrame(KeyFrame *pKF);
        void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

    private:

        float mKeyFrameSize;
        float mKeyFrameLineWidth;
        float mGraphLineWidth;
        float mPointSize;
        float mCameraSize;
        float mCameraLineWidth;

        cv::Mat mCameraPose;

        std::mutex mMutexCamera;
    };

} //namespace ORB_SLAM

#endif //ORB_VINS_SLAM_MAPDRAWER_H
