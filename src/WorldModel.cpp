/**
 * @file   WorldModel.cpp
 * @brief  WorldModel definitions.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

// my includes
#include "WorldModel.h"

#include "SFPlot.h"

#include <Eigen/Geometry>
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
#include <fstream>

using namespace gtsam;
using namespace anloro;

WorldModel* WorldModel::worldModel_= nullptr;

anloro::WorldModel::WorldModel()
{
    _plotterThread = new std::thread(&WorldModelPlotter::Spin, &_plotter);
}

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
    // Add to the render window
    InsertKeyFrameToPlot(keyFrame);
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
    Eigen::Matrix3d n;
    int idFrom, idTo, NodeId;
    double x, y, z, roll, pitch, yaw, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
    for (std::map<int, PoseFactor *>::const_iterator iter = _poseFactorsMap.begin(); iter != _poseFactorsMap.end(); ++iter)
    {
        idFrom = iter->second->From();
        idTo = iter->second->To();
        iter->second->GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
        iter->second->GetEulerVariances(sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);
        auto noiseModel = noiseModel::Diagonal::Sigmas((Vector(6) << sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw).finished());
        n = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) 
          * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) 
          * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Rot3 newR = Rot3(n);
        // Rot3 newR = Rot3().Yaw(yaw).Pitch(pitch).Roll(roll);
        Point3 newP = Point3(x, y, z);
        Pose3 newMean = Pose3(newR, newP);
        graph.emplace_shared<BetweenFactor<Pose3>>(idFrom, idTo, newMean, noiseModel);
    }

    // Add the nodes
    for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter != _keyFramesMap.end(); ++iter)
    {
        NodeId = iter->first;
        iter->second->GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
        n = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) 
          * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) 
          * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Rot3 newR = Rot3(n);
        Point3 newP = Point3(x, y, z);
        Pose3 newPose = Pose3(newR, newP);
        initialEstimate.insert(NodeId, newPose);
    }

    graph.print("This is the graph:\n");
    initialEstimate.print("This is the initial estimate:\n");
    GaussNewtonParams parameters;
    // Stop iterating once the change in error between steps is less than this value
    parameters.relativeErrorTol = 1e-5;
    // Do not perform more than N iteration steps
    parameters.maxIterations = 100;
    GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
    double err0 = optimizer.error();
    std::cout << "The error is: " << err0 << std::endl;
    optimizedPoses = optimizer.optimize();
    double err = optimizer.error();
    std::cout << "The error is: " << err << std::endl;

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

    UpdateCompletePlot();
}

void anloro::WorldModel::SavePosesRaw()
{
    // Create and open a text file
    std::ofstream outputFile("rawposes.txt");

    int nodeId;
    double x, y, z, roll, pitch, yaw;
    // Iterate over the KeyFrame's map
    for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter != _keyFramesMap.end(); ++iter)
    {   
        // Get the information of each node
        nodeId = iter->first;
        iter->second->GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
        
        // Write to the file
        outputFile << nodeId << " " << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl;
    }

    std::cout << "Raw poses saved as rawposes.txt" << std::endl;

    // Close the file
    outputFile.close();
}

void anloro::WorldModel::InsertKeyFrameToPlot(KeyFrame<int> *keyFrame)
{
    double x, y, z, roll, pitch, yaw;

    std::vector<float> *xAxis = _plotter.GetxAxis();
    std::vector<float> *yAxis = _plotter.GetyAxis();
    keyFrame->GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
    xAxis->push_back(x);
    yAxis->push_back(y);
}

void anloro::WorldModel::UpdateCompletePlot()
{
    double x, y, z, roll, pitch, yaw;

    std::vector<float> *newxAxis = new std::vector<float>{};
    std::vector<float> *newyAxis = new std::vector<float>{};
    // Iterate over the KeyFrame's map
    for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter != _keyFramesMap.end(); ++iter)
    {
        // Get the information of each node
        iter->second->GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
        newxAxis->push_back(x);
        newyAxis->push_back(y);
    }

    _plotter.SetAxis(newxAxis, newyAxis);    
}