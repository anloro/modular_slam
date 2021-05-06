/**
 * @file   WorldModel.cpp
 * @brief  WorldModel definitions.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

// my includes
#include "WorldModel.h"

// includes to create the custom graph
// includes to the gtsam graph
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/sam/BearingFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose2.h>

#include <iostream>
#include <chrono>

using namespace gtsam;
using namespace anloro;

WorldModel* WorldModel::worldModel_= nullptr;

WorldModel * WorldModel::GetInstance()
{
    if (worldModel_ == nullptr)
    {
        worldModel_ = new WorldModel();
    }
    return worldModel_;

    // static WorldModel* instance; // a static local variable is initialized on first call to the
    //                              // function only, afterwards, it is always the same variable
    //                              // for every subsequent calls to the function.
    // return instance;
};

// ---------------------------------------------------------
// --------- Utilities for Front-end interaction -----------
// ---------------------------------------------------------

// Map the input ID into our own internal IDs
int anloro::WorldModel::InternalMapId(int id)
{
    // TO-DO: Add a time check to comapre between ids of different front-ends!

    static int count = 0; // initialized only once across all calls
    // Keep track of the maps
    if (_rtabToModular.count(id) == 0)
    {
        // ID not found, so add it
        _rtabToModular.insert(std::make_pair(id, count));
        _modularToRtab.insert(std::make_pair(count, id));
        return count++;
    }
    else
    {
        // ID found
        return _rtabToModular[id];
    }
}

// ---------------------------------------------------------
// ------------- Entity creation definitions ---------------
// ---------------------------------------------------------

// Add a Reference Frame Entity to the graph
void anloro::WorldModel::AddRefFrameEntity(RefFrame *refFrame)
{
    static int refFrameID = 0; // initialized only once across all calls
    _refFramesMap.insert(RefFramePair(refFrameID, refFrame));
    refFrameID++;
}

// Add a Key-Frame Entity to the graph
void anloro::WorldModel::AddKeyFrameEntity(int id, KeyFrame<int> *keyFrame)
{
    _keyFramesMap.insert(KeyFramePair(id, keyFrame));
}

// ---------------------------------------------------------
// ------------- Factor creation definitions ---------------
// ---------------------------------------------------------

// Add a pose Factor to the graph
void anloro::WorldModel::AddPoseFactor(PoseFactor *poseFactor)
{
    static int poseFactorID = 0; // initialized only once across all calls
    _poseFactorsMap.insert(PoseFactorPair(poseFactorID, poseFactor));
    poseFactorID++;
}

// ---------------------------------------------------------
// ---------------- Interface with Gtsam -------------------
// ---------------------------------------------------------

// Optimize the World model (using gtsam)
void anloro::WorldModel::Optimize()
{
    NonlinearFactorGraph graph;
    Values initialEstimate;
    Values optimizedPoses;

    // Add a prior on the first pose, setting it to the origin
    int initId = 0;
    Pose2 priorPose2 = Pose2(0, 0, 0);
    Pose3 priorPose = Pose3(priorPose2);
    auto priorNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.3, 0.3, 0.0, 0.0, 0.0, 0.1).finished());
    graph.addPrior(initId, priorPose, priorNoise);

    // Add the Pose3 factors
    int idFrom, idTo, NodeId;
    double x, y, z, roll, pitch, yaw, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
    for (std::map<int, PoseFactor *>::const_iterator iter = _poseFactorsMap.begin(); iter != _poseFactorsMap.end(); ++iter)
    {
        idFrom = iter->second->From();
        idTo = iter->second->To();
        iter->second->GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
        iter->second->GetEulerVariances(sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);
        auto noiseModel = noiseModel::Diagonal::Sigmas((Vector(6) << sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw).finished());
        Rot3 newR = Rot3().Yaw(yaw).Pitch(pitch).Roll(roll);
        Point3 newP = Point3(x, y, z);
        Pose3 newMean = Pose3(newR, newP);
        graph.emplace_shared<BetweenFactor<Pose3>>(idFrom, idTo, newMean, noiseModel);
    }

    // Add the nodes
    for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter != _keyFramesMap.end(); ++iter)
    {
        NodeId = iter->first;
        iter->second->GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
        Rot3 newR = Rot3().Yaw(yaw).Pitch(pitch).Roll(roll);
        Point3 newP = Point3(x, y, z);
        Pose3 newPose = Pose3(newR, newP);
        initialEstimate.insert(NodeId, newPose);
    }

    graph.print();

    GaussNewtonParams parameters;
    GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
    optimizedPoses = optimizer.optimize();

    // This is for testing
    optimizedPoses.print("Optimized poses:\n");
    // Calculate and print marginal covariances for all variables
    std::cout.precision(3);
    Marginals marginals(graph, optimizedPoses);


    // for (Values::const_iterator iter = optimizer.values().begin(); iter != optimizer.values().end(); ++iter)

    // Update the World Model with the optimized poses.
    for (std::pair<Values::const_iterator, std::map<int, KeyFrame<int> *>::const_iterator> iter(optimizer.values().begin(), _keyFramesMap.begin());
         iter.first != optimizer.values().end() && iter.second != _keyFramesMap.end();
         ++iter.first, ++iter.second)
    {
        // First get the optimized poses from the optimizer in Euler format
        int key = (int)iter.first->key;
        Pose3 optimizedPose = iter.first->value.cast<Pose3>();
        x = optimizedPose.x();
        y = optimizedPose.y();
        z = optimizedPose.z();
        Rot3 r = optimizedPose.rotation();
        Vector3 eulerR = r.xyz();
        roll = eulerR[0];
        pitch = eulerR[1];
        yaw = eulerR[2];
        
        // In case we need the marginals
        // std::cout << "Key-Frame " << key << " has a covariance of:\n"
        //           << marginals.marginalCovariance(key) << std::endl;

        // Then we update the World Model with the optimized poses
        iter.second->second->SetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
    }

}
