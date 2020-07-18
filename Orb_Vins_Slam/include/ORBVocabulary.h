//
// Created by flk on 20-4-2.
//

#ifndef ORB_VINS_SLAM_ORBVOCABULARY_H
#define ORB_VINS_SLAM_ORBVOCABULARY_H


#include "Thirdparty/DBoW2/DBoW2/FORB.h"
#include "Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"
//#include "frame.h"

namespace ovslam
{

    typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
            ORBVocabulary;

}

#endif //ORB_VINS_SLAM_ORBVOCABULARY_H
