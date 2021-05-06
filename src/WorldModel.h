/**
 * @file   WorldModel.h
 * @brief  World model declarations and templates.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#pragma once

// my includes
#include "KeyFrame.h"
#include "PoseFactor.h"

#include <boost/any.hpp>

// includes to gtsam 
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include <iostream>
#include <chrono>


namespace anloro{

class WorldModel
{
    private:
        // CONSTRUCTOR (private to make it a singleton)
        WorldModel() {};
    
    protected:
        static WorldModel* worldModel_;

    public:
        // Singletons should not be cloneable.
        WorldModel(WorldModel const&) = delete;
        // Singletons should not be assignable.
        void operator = (WorldModel const&) = delete;
        // WorldModel(WorldModel const &) = delete;
        // void operator=(WorldModel const &) = delete;

        // MEMBER FUNCTIONS
        // Public static function returning a reference to the singleton class:
        static WorldModel *GetInstance();
        // Front-end utilities
        int InternalMapId(int id);
        // Entity creation
        void AddRefFrameEntity(RefFrame * refFrame);
        void AddKeyFrameEntity(int nodeId, KeyFrame<int> * keyFrame);
        // Factor creation
        void AddPoseFactor(PoseFactor * poseFactor);
        // Optimization
        void Optimize();

        // DEFINITIONS FOR EASIER READABILITY
        typedef std::map<int, RefFrame*> RefFramesMap;
        typedef std::pair<int, RefFrame*> RefFramePair;
        typedef std::map<int, KeyFrame<int>*> KeyFramesMap;
        typedef std::pair<int, KeyFrame<int>*> KeyFramePair;
        typedef std::map<int, PoseFactor*> PoseFactorsMap;
        typedef std::pair<int, PoseFactor*> PoseFactorPair;

    protected :
        // MAPS DATA 
        RefFramesMap _refFramesMap;
        KeyFramesMap _keyFramesMap;
        PoseFactorsMap _poseFactorsMap;

        // RTABMAP'S INTERFACE DATA
        // Map Rtab-Map id into this framework id
        std::map<int, int> _rtabToModular;
        // Map this framework id into Rtab-Map id
        std::map<int, int> _modularToRtab;

};


} // namespace anloro
