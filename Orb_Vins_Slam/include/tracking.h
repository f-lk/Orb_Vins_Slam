//
// Created by flk on 20-4-2.
//

#ifndef ORB_VINS_SLAM_TRACKING_H
#define ORB_VINS_SLAM_TRACKING_H

#include "frame.h"
#include "initializer.h"
#include "ORBmatcher.h"
#include "system.h"
#include "keyframedatabase.h"
#include "ORBVocabulary.h"
#include "optimizer.h"
#include "pnpsolver.h"
#include "keyframe.h"

namespace ovslam
{

class System;
class ORBextractor;
//class Frame;
class Viewer;
class FrameDrawer;
class MapDrawer;
class Map;
class KeyFrame;


    class Tracking
    {
    public:
        Tracking();

        cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

        Tracking(System *pSys,  Map *pMap,  const string &strSettingPath, FrameDrawer *pFrameDrawer,
                MapDrawer *pMapDrawer, KeyFrameDatabase* pKFDB, ORBVocabulary* pVoc,const int sensor);
        void SetViewer(Viewer *pViewer);

        bool TrackReferenceKeyFrame();

        bool TrackLocalMap();

        void UpdateLocalMap();

        void SearchLocalPoints();



        ///////////////////////////////////////////////////////////////////////////
        enum eTrackingState{
            SYSTEM_NOT_READY=-1,  //
            NO_IMAGES_YET=0,
            NOT_INITIALIZED=1,    //没有初始化
            OK=2,                 //正常
            LOST=3                //跟丢
        };


        // Input sensor:MONOCULAR, STEREO, RGBD
        int mSensor;

        // True if local mapping is deactivated and we are performing only localization
        bool mbOnlyTracking;

        eTrackingState mState;

        eTrackingState mLastProcessedState;

        // Initialization Variables (Monocular)
        // 初始化时前两帧相关变量
        std::vector<int> mvIniLastMatches;
        std::vector<int> mvIniMatches;// 跟踪初始化时前两帧之间的匹配
        std::vector<cv::Point2f> mvbPrevMatched;
        std::vector<cv::Point3f> mvIniP3D;
        Frame mInitialFrame;

        cv::Mat mImGray;

        bool mbRGB;

//        ORBextractor * mpIniORBextractor;

        cv::Mat mK;
        cv::Mat mDistCoef;
        float mbf;

        // Threshold close/far points
        // Points seen as close by the stereo/RGBD sensor are considered reliable
        // and inserted from just one frame. Far points requiere a match in two keyframes.
        float mThDepth;

        // Current Frame
        Frame mCurrentFrame;

        list<cv::Mat> mlRelativeFramePoses;







        //private  和  protected 的区别，派生类的成员可以直接访问基类的保护成员，但不能直接访问基类的私有成员。
    private:



    protected:
        void Track();

        void MonocularInitialization();

        void CreateInitialMapMonocular();

        void UpdateLocalKeyFrames();

        bool TrackWithMotionModel();

        void UpdateLastFrame();

        bool Relocalization();

        void UpdateLocalPoints();

        bool NeedNewKeyFrame();

        void CreateNewKeyFrame();




        /////////////////////////////////////////////////////////////////////////////////////////////////

        // Initalization (only for monocular)
        // 单目初始器
        Initializer* mpInitializer;

        Map* mpMap;

//        KeyFrameDatabase* mpKeyFrameDB;

        //LocalMapping* mpLocalMapper;

        //Last Frame, KeyFrame and Relocalisation Info
        unsigned int mnLastKeyFrameId;
        unsigned int mnLastRelocFrameId;
        KeyFrame* mpLastKeyFrame;

        //Local Map
        KeyFrame* mpReferenceKF;// 当前关键帧就是参考帧
        std::vector<KeyFrame*> mvpLocalKeyFrames;
        std::vector<MapPoint*> mvpLocalMapPoints;


        Frame mLastFrame;

        // In case of performing only localization, this flag is true when there are no matches to
        // points in the map. Still tracking will continue if there are enough matches with temporal points.
        // In that case we are doing visual odometry. The system will try to do relocalization to recover
        // "zero-drift" localization to the map.
        bool mbVO;

        // System
        System* mpSystem;

        //New KeyFrame rules (according to fps)
        int mMinFrames;
        int mMaxFrames;


        // ORB
        // orb特征提取器，不管单目还是双目，mpORBextractorLeft都要用到
        // 如果是双目，则要用到mpORBextractorRight
        // 如果是单目，在初始化的时候使用mpIniORBextractor而不是mpORBextractorLeft，
        // mpIniORBextractor属性中提取的特征点个数是mpORBextractorLeft的两倍
        ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
        ORBextractor* mpIniORBextractor;

        Viewer* mpViewer;
        FrameDrawer* mpFrameDrawer;
        MapDrawer* mpMapDrawer;

        ORBVocabulary* mpORBVocabulary;
        KeyFrameDatabase* mpKeyFrameDB;

        //Other Thread Pointers
//        LocalMapping* mpLocalMapper;
//        LoopClosing* mpLoopClosing;

        //Motion Model
        cv::Mat mVelocity;

        list<MapPoint*> mlpTemporalPoints;

        //Current matches in frame
        int mnMatchesInliers;



    };
}




#endif //ORB_VINS_SLAM_TRACKING_H
