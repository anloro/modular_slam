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
        void AddEntityLandMark(int nodeId, double x, double y, double z);
        void AddEntityRefFrame(int nodeId, double x, double y, double z, double roll, double pitch, double yaw);
        // void AddEntityKeyFrame(int nodeId, double x, double y, double theta);
        // void AddEntityKeyFrame(int nodeId, double x, double y, double z, double roll, double pitch, double yaw);
        template <class... Ts> 
        void AddEntityKeyFrame(int nodeId, double x, double y, double z, double roll, double pitch, double yaw, Ts ...data);
        // Factor creation
        void AddFactor(int fromNode, int toNode, double x, double y, double theta, double sigmaX, double sigmaY, double sigmaTheta);
        void AddFactor(int fromNode, int toNode, double x, double y, double z, double roll, double pitch, double yaw, double sigmaX, double sigmaY, double sigmaZ, double sigmaRoll, double sigmaPitch, double sigmaYaw);
        // Optimization
        void Optimize();

    // private :
        // save the info of the custom pose graph
        // std::vector<Entity> _entities;
        // Factor _factors;

        // save the info for gtsam
        gtsam::NonlinearFactorGraph _graph;
        gtsam::Values _initialEstimate;
        gtsam::Values _result;
        // std::map<int, KeyFrame> _myKeyFrameMap;
};

template <class... Ts>
void WorldModel::AddEntityKeyFrame(int nodeId, double x, double y, double z, double roll, double pitch, double yaw, Ts ...data)
{
    KeyFrame<Ts...> newKeyFrame;
    newKeyFrame = KeyFrame<Ts...>(x, y, z, roll, pitch, yaw, std::forward<Ts...>(data)...);
    AddInitialEstimate3ToGtsam(nodeId, x, y, z, roll, pitch, yaw);
}