//
// Created by flk on 20-4-2.
//

#ifndef ORB_VINS_SLAM_MAP_H
#define ORB_VINS_SLAM_MAP_H

#include "mappoint.h"
#include "keyframe.h"
#include "frame.h"
#include <set>

#include <mutex>



namespace ovslam
{

    class MapPoint;
    class KeyFrame;

    class Map
    {
    public:
        Map();

        void AddKeyFrame(KeyFrame* pKF);
        void AddMapPoint(MapPoint* pMP);
        void EraseMapPoint(MapPoint* pMP);
        void EraseKeyFrame(KeyFrame* pKF);
        void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

        std::vector<KeyFrame*> GetAllKeyFrames();
        std::vector<MapPoint*> GetAllMapPoints();
        std::vector<MapPoint*> GetReferenceMapPoints();

        long unsigned int MapPointsInMap();
        long unsigned  KeyFramesInMap();

        long unsigned int GetMaxKFid();

        void clear();

        std::vector<KeyFrame*> mvpKeyFrameOrigins;

        std::mutex mMutexMapUpdate;

        // This avoid that two points are created simultaneously in separate threads (id conflict)
        std::mutex mMutexPointCreation;

    protected:
        std::set<MapPoint*> mspMapPoints; ///< MapPoints
        std::set<KeyFrame*> mspKeyFrames; ///< Keyframs

        std::vector<MapPoint*> mvpReferenceMapPoints;

        long unsigned int mnMaxKFid;

        std::mutex mMutexMap;
    };

}



#endif //ORB_VINS_SLAM_MAP_H
