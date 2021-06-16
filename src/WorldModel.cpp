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

#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>

using namespace gtsam;
using namespace anloro;


WorldModel *WorldModel::worldModel_ = nullptr;
int anloro::WorldModel::currentNodeId = 0;

anloro::WorldModel::WorldModel()
{
    _plotterThread = new std::thread(&WorldModelPlotter::Spin, &_plotter);
}

WorldModel *WorldModel::GetInstance()
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
    UpdateCompletePlot();
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

    Transform transform;
    int idFrom, idTo, NodeId;

    // Add the Pose3 factors
    float sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
    for (std::map<int, PoseFactor *>::const_iterator iter = _poseFactorsMap.begin(); iter != _poseFactorsMap.end(); ++iter)
    {
        idFrom = iter->second->From();
        idTo = iter->second->To();
        transform = iter->second->GetTransform();

        iter->second->GetEulerVariances(sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);
        auto noiseModel = noiseModel::Diagonal::Variances((Vector(6) << sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw).finished());
        Pose3 newMean = Pose3(transform.ToMatrix4f().cast<double>()); // Pose3 needs a double datatype

        graph.emplace_shared<BetweenFactor<Pose3>>(idFrom, idTo, newMean, noiseModel);
    }

    // Add the nodes
    for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter != _keyFramesMap.end(); ++iter)
    {
        NodeId = iter->first;
        transform = iter->second->GetTransform();
        Pose3 newPose = Pose3(transform.ToMatrix4f().cast<double>()); // Pose3 needs a double datatype
        initialEstimate.insert(NodeId, newPose);
    }


    // GaussNewtonParams parameters;
    LevenbergMarquardtParams parameters;
    int maxIterations = 100; 
    float relativeErrorTol = 1e-5; // Stop iterating if the error is below this value
    parameters.relativeErrorTol = relativeErrorTol;
    parameters.maxIterations = maxIterations;
    // GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, parameters);
    float iniError = optimizer.error();
    std::cout << "The initial error is: " << iniError << std::endl;

    // TRY gtsam approach in RTAB-Map
    int it = 0;
    double lastError = optimizer.error();
    for (int i = 0; i < maxIterations; ++i)
    {
        try
        {
            optimizer.iterate();
            ++it;
        }
        catch (gtsam::IndeterminantLinearSystemException &e)
        {
            std::cout << e.what() << std::endl;
        }
        // early stop condition
        double error = optimizer.error();
        double errorDelta = lastError - error;
        if (i > 0 && errorDelta < relativeErrorTol)
        {
            if (errorDelta < 0)
            {
                // Negative improvement?! Ignore and continue optimizing...
            }
            else
            {
                // Not enough improvement, stop optimization.
                break;
            }
        }
        else if (i == 0 && error < relativeErrorTol)
        {
            // Error is already under relativeErrorTol, stop optimization.
            break;
        }
        lastError = error;
        std::cout << "The error after the iteration " << i << " is: " << error << std::endl;
    }

    // optimizedPoses = optimizer.optimize();
    // float endError = optimizer.error();
    // std::cout << "The error after complete optimization is: " << endError << std::endl;

    int graphKey, worldKey;
    // Update the World Model with the optimized poses.
    for (std::pair<Values::const_iterator, std::map<int, KeyFrame<int> *>::const_iterator> iter(optimizer.values().begin(), _keyFramesMap.begin());
         iter.first != optimizer.values().end() && iter.second != _keyFramesMap.end();
         ++iter.first, ++iter.second)
    {
        // Get the optimized poses from the optimizer
        Pose3 optimizedPose = iter.first->value.cast<Pose3>();
        Eigen::Matrix4f matrix = optimizedPose.matrix().cast<float>();
        Transform transform = Transform(matrix);

        // Check if the iterator is in the same node.
        graphKey = (int)iter.first->key;
        worldKey = iter.second->first;
        if (graphKey == worldKey)
        {
            // Update the World Model with the optimized poses
            iter.second->second->SetTransform(transform);
        }
        else
        {
            std::cout << "ERROR: The World model update is failing!" << std::endl;
        }
    }

    UpdateCompletePlot();
}

// Return the optimized poses map
std::map<int, Eigen::Affine3f> anloro::WorldModel::GetOptimizedPoses()
{
    int nodeId, frontEndId;
    Transform transform;
    std::map<int, Eigen::Affine3f> optimizedPoses;
    typedef std::pair<int, Eigen::Affine3f> optimizedPose;

    // Iterate over the KeyFrame's map
    for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter != _keyFramesMap.end(); ++iter)
    {
        // Get the information of each node
        nodeId = iter->first;
        transform = iter->second->GetTransform();
        // frontEndId = GetIdFromInternalMap(nodeId);

        optimizedPoses.insert(optimizedPose(nodeId, transform.GetAffineTransform()));
    }

    return optimizedPoses;
}

void anloro::WorldModel::UndoOdometryCorrection(int lastLoopId, Eigen::Matrix4f uncorrection)
{
    // int truid = InternalMapId(lastLoopId);
    Transform transform, newTransform;
    Eigen::Matrix4f uncorrected;

    // Iterate over the KeyFrame's map
    for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter->first != lastLoopId; ++iter)
    {
        transform = iter->second->GetTransform();
        uncorrected = uncorrection * transform.ToMatrix4f();
        newTransform = Transform(uncorrected);

        iter->second->SetTransform(newTransform);
    }
}


void anloro::WorldModel::SavePosesRaw()
{
    // Create and open a text file
    std::ofstream outputFile("rawposes.txt");

    int nodeId;
    float x, y, z, roll, pitch, yaw;
    Transform transform;

    // Iterate over the KeyFrame's map
    for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter != _keyFramesMap.end(); ++iter)
    {
        // Get the information of each node
        nodeId = iter->first;
        transform = iter->second->GetTransform();
        transform.GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);

        // Write to the file
        outputFile << nodeId << " " << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl;
    }

    std::cout << "Raw poses saved as rawposes.txt" << std::endl;

    // Close the file
    outputFile.close();
}

void anloro::WorldModel::InsertKeyFrameToPlot(KeyFrame<int> *keyFrame)
{
    float x, y, z, roll, pitch, yaw;

    std::vector<float> *xAxis = _plotter.GetxAxis();
    std::vector<float> *yAxis = _plotter.GetyAxis();
    Transform transform = keyFrame->GetTransform();
    transform.GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
    xAxis->push_back(x);
    yAxis->push_back(y);
}

void anloro::WorldModel::UpdateCompletePlot()
{
    float x, y, z, roll, pitch, yaw;

    std::vector<float> *newxAxis = new std::vector<float>{};
    std::vector<float> *newyAxis = new std::vector<float>{};
    Transform transform;
    // Iterate over the KeyFrame's map
    for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter != _keyFramesMap.end(); ++iter)
    {
        // Get the information of each node
        transform = iter->second->GetTransform();
        transform.GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
        newxAxis->push_back(x);
        newyAxis->push_back(y);
    }

    _plotter.SetAxis(newxAxis, newyAxis);
}