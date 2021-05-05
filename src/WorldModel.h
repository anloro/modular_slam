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
    public:
        // Constructor
        WorldModel();

        // Member functions
        void AddInitialEstimate3ToGtsam(int nodeId, double x, double y, double z, double roll, double pitch, double yaw);
        // Entity creation
        void AddRefFrameEntity(RefFrame * refFrame);
        void AddKeyFrameEntity(int nodeId, KeyFrame<int> * keyFrame);
        void AddPoseFactor(PoseFactor * poseFactor);
        template <typename T>
        void AddEntity(int id, T entity);
        template <typename T>
        T GetEntity(int id);
        // Factor creation
        void AddPoseFactor(int fromNode, int toNode, double x, double y, double theta, double sigmaX, double sigmaY, double sigmaTheta);
        void AddPoseFactor(int fromNode, int toNode, double x, double y, double z, double roll, double pitch, double yaw, double sigmaX, double sigmaY, double sigmaZ, double sigmaRoll, double sigmaPitch, double sigmaYaw);
        void AddRangeOnlyFactor(int fromNode, int toNode, double range, double sigmaRange);
        // void AddBearingOnlyFactor(int fromNode, int toNode, double bearing, double sigmaBearing);
        int InternalMapId(int id);

        // Optimization
        void Optimize();

        // Definitions for easier readability
        typedef std::map<int, RefFrame*> RefFramesMap;
        typedef std::pair<int, RefFrame*> RefFramePair;
        typedef std::map<int, KeyFrame<int>*> KeyFramesMap;
        typedef std::pair<int, KeyFrame<int>*> KeyFramePair;
        typedef std::map<int, PoseFactor*> PoseFactorsMap;
        typedef std::pair<int, PoseFactor*> PoseFactorPair;

    protected :
        // Map 
        std::map<int, boost::any> _myMap;
        RefFramesMap _refFramesMap;
        KeyFramesMap _keyFramesMap;
        PoseFactorsMap _poseFactorsMap;

        // RTABMAP'S INTERFACE DATA
        // Map Rtab-Map id into this framework id
        std::map<int, int> _rtabToModular;
        // Map this framework id into Rtab-Map id
        std::map<int, int> _modularToRtab;
        // GTSAM'S INTERFACE DATA
        gtsam::NonlinearFactorGraph _graph;
        gtsam::Values _initialEstimate;
        gtsam::Values _result;
};

template <typename T>
void WorldModel::AddEntity(int id, T entity)
{
    int internalId;
    internalId = InternalMapId(id);
    // Add the entity to the global map
    _myMap.insert(std::make_pair(internalId, entity));
    std::cout << "The id " << id << " maps into internal id " << internalId << std::endl;
}

template <typename T>
T WorldModel::GetEntity(int id)
{
    return boost::any_cast<T>(_myMap[id]);
}

} // namespace anloro
