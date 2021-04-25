/**
 * @file   WorldModel.h
 * @brief  Defines the Key-Frame Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#pragma once
// my includes
// #include "Entity.h"
#include "KeyFrame.h"
#include <boost/any.hpp>

// includes to create the custom graph
#include <opencv2/core/core.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
// includes to the gtsam graph
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
// #include <gtsam/geometry/Pose2.h>
// #include <gtsam/inference/Key.h>
// #include <gtsam/nonlinear/Marginals.h>

#include <iostream>
#include <chrono>

    class WorldModel
{
    public:
        // Constructor
        WorldModel();

        void AddInitialEstimate3ToGtsam(int nodeId, double x, double y, double z, double roll, double pitch, double yaw);
        // Entity creation
        template <typename T>
        void AddEntity(int id, T entity);
        template <typename T>
        T GetEntity(int id);
        // void AddEntityLandMark(int nodeId, double x, double y, double z);
        // void AddEntityRefFrame(int nodeId, double x, double y, double z, double roll, double pitch, double yaw);
        // void AddEntityKeyFrame(int nodeId, double x, double y, double theta);
        // void AddEntityKeyFrame(int nodeId, double x, double y, double z, double roll, double pitch, double yaw);
        // template <class... Ts> 
        // void AddEntityKeyFrame(int nodeId, double x, double y, double z, double roll, double pitch, double yaw, Ts ...data);
        // Factor creation
        void AddPoseFactor(int fromNode, int toNode, double x, double y, double theta, double sigmaX, double sigmaY, double sigmaTheta);
        void AddPoseFactor(int fromNode, int toNode, double x, double y, double z, double roll, double pitch, double yaw, double sigmaX, double sigmaY, double sigmaZ, double sigmaRoll, double sigmaPitch, double sigmaYaw);
        // void AddBearingOnlyFactor(int fromNode, int toNode, double bearing, double sigmaBearing);
        void AddRangeOnlyFactor(int fromNode, int toNode, double range, double sigmaRange);

        // Optimization
        void Optimize();

    // private :
        // Map 
        std::map<int, boost::any> _myMap;
        // Data for gtsam
        gtsam::NonlinearFactorGraph _graph;
        gtsam::Values _initialEstimate;
        gtsam::Values _result;
};

template <typename T>
void WorldModel::AddEntity(int id, T entity)
{
    _myMap.insert(std::make_pair(id, entity));
}

template <typename T>
T WorldModel::GetEntity(int id)
{
    return boost::any_cast<T>(_myMap[id]);
}

// template <class... Ts>
// void WorldModel::AddEntityKeyFrame(int nodeId, double x, double y, double z, double roll, double pitch, double yaw, Ts ...data)
// {
//     KeyFrame<Ts...> newKeyFrame;
//     newKeyFrame = KeyFrame<Ts...>(x, y, z, roll, pitch, yaw, std::forward<Ts...>(data)...);
//     AddInitialEstimate3ToGtsam(nodeId, x, y, z, roll, pitch, yaw);
// }