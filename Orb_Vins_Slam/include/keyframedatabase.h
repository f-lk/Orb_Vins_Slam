//
// Created by flk on 20-4-5.
//

#ifndef ORB_VINS_SLAM_KEYFRAMEDATABASE_H
#define ORB_VINS_SLAM_KEYFRAMEDATABASE_H


#include <vector>
#include <list>
#include <set>

#include "keyframe.h"
#include "frame.h"
#include "ORBVocabulary.h"

#include<mutex>


namespace ovslam
{

    class KeyFrame;
//    class Frame;


    class KeyFrameDatabase
    {
    public:

        KeyFrameDatabase(const ORBVocabulary &voc);

        void add(KeyFrame* pKF);

        void erase(KeyFrame* pKF);

        void clear();

        // Loop Detection
        std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

        // Relocalization
        std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

    protected:

        // Associated vocabulary
        const ORBVocabulary* mpVoc; ///< 预先训练好的词典

        // Inverted file
        std::vector<list<KeyFrame*> > mvInvertedFile; ///< 倒排索引，mvInvertedFile[i]表示包含了第i个word id的所有关键帧

        // Mutex
        std::mutex mMutex;
    };

}




#endif //ORB_VINS_SLAM_KEYFRAMEDATABASE_H
