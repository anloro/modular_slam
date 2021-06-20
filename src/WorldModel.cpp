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
#include <gtsam/sam/BearingRangeFactor.h>

#include <iostream>
#include <chrono>
#include <fstream>
#include <math.h>

#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>

using namespace gtsam;
using namespace anloro;


WorldModel *WorldModel::worldModel_ = nullptr;
// int anloro::WorldModel::currentNodeId = -1;
// int anloro::WorldModel::currentLandMarkId = -1;

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

// Add a Landmark Entity to the graph
void anloro::WorldModel::AddLandMarkEntity(int id, LandMark *LandMark)
{
    _landMarksMap.insert(LandMarkPair(id, LandMark));
}

// Get an existing LandMark pointer from the map
LandMark* anloro::WorldModel::GetLandMarkEntity(int id)
{
    return _landMarksMap[id];
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
    int idFrom, idTo, NodeId, landMarkId;

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

    // Add the landmarks
    float x, y, z, dummyVar;
    int nodeId;
    for (std::map<int, LandMark*>::const_iterator iteri = _landMarksMap.begin(); iteri != _landMarksMap.end(); ++iteri)
    {
        landMarkId = iteri->first;
        Symbol landMarkKey('l', landMarkId);

        LandMark::RelatedNodesMap relatedNodes = iteri->second->GetRelatedNodes();

        for (LandMark::RelatedNodesMap::const_iterator iterj = relatedNodes.begin(); iterj != relatedNodes.end(); ++iterj)
        {
            nodeId = iterj->first;
            transform = std::get<0>(iterj->second);
            LandMark::Uncertainty unc = std::get<1>(iterj->second);
            sigmaX = unc[0];
            sigmaY = unc[1];
            sigmaZ = unc[2];
            sigmaRoll = unc[3];
            sigmaPitch = unc[4];
            sigmaYaw = unc[5];

            // auto measurementNoise = noiseModel::Diagonal::Variances((Vector(6) << sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw).finished());
            // auto measurementNoise = noiseModel::Diagonal::Variances((Vector(3) << sigmaX, sigmaY, sigmaZ).finished());
            auto measurementNoise = noiseModel::Diagonal::Variances((Vector(3) << 0.2, 0.2, 0.2).finished());
            
            // Add transform as a Pose3 BetweenFactor
            // Pose3 newMean = Pose3(transform.ToMatrix4f().cast<double>()); // Pose3 needs a double datatype
            // graph.emplace_shared<BetweenFactor<Pose3>>(nodeId, landMarkKey, newMean, measurementNoise);

            // Add transform as a Point3 BetweenFactor
            // transform.GetTranslationalAndEulerAngles(x, y, z, dummyVar, dummyVar, dummyVar);
            // Point3 pLandMark = Point3(x, y, z);
            // graph.emplace_shared<BetweenFactor<Point3>>(nodeId, landMarkKey, pLandMark, measurementNoise);

            // Add transform as a bearing range factor
            // Let's convert the point into polar coordinates for GTSAM 
            transform.GetTranslationalAndEulerAngles(x, y, z, dummyVar, dummyVar, dummyVar);
            float range = std::sqrt(x*x + y*y + z*z);
            // float theta = atan(y/x);
            // float sigma = atan(std::sqrt(x*x + y*y)/z);
            // Eigen::Matrix3f rot;
            // rot = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(sigma, Eigen::Vector3f::UnitZ());
            // Rot3 rot3 = Rot3(rot.cast<double>());
            Unit3 u3 = Unit3(x, y, z);
            graph.emplace_shared<BearingRangeFactor<Pose3, Point3> >(nodeId, landMarkKey, u3, range, measurementNoise);

        }

        // Add the initial estimate as the last node registered (should it be used the mean?)
        // As a Pose3 
        // Eigen::Matrix4f lm_abs_coord = _keyFramesMap.at(nodeId)->GetTransform().ToMatrix4f() * transform.ToMatrix4f();
        // Pose3 newMean_abs_coord = Pose3(lm_abs_coord.cast<double>());
        // initialEstimate.insert(landMarkKey, newMean_abs_coord);
        // As a Point3
        // Transform lm_abs_coord = Transform(_keyFramesMap.at(nodeId)->GetTransform().ToMatrix4f() * transform.ToMatrix4f());
        // lm_abs_coord.GetTranslationalAndEulerAngles(x, y, z, dummyVar, dummyVar, dummyVar);
        
        // The initial estimate is in absolute coordinates (map coordinate frame) 
        // this is computed by using the transform of the last visited node from which
        // the landmark was detected (may be better to use the mean of every node in which it was detected)
        float nodeX, nodeY, nodeZ;
        _keyFramesMap.at(nodeId)->GetTransform().GetTranslationalAndEulerAngles(nodeX, nodeY, nodeZ, dummyVar, dummyVar, dummyVar);
        Point3 pLandMark_abs_coord = Point3(nodeX + x, nodeY + y, nodeZ + z);
        initialEstimate.insert(landMarkKey, pLandMark_abs_coord);

    }

    graph.print();
    initialEstimate.print();

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
        // Check if the iterator is in the same node.
        graphKey = (int)iter.first->key;
        worldKey = iter.second->first;
        if (graphKey == worldKey)
        {
            // Get the optimized poses from the optimizer
            Pose3 optimizedPose = iter.first->value.cast<Pose3>();
            Eigen::Matrix4f matrix = optimizedPose.matrix().cast<float>();
            Transform transform = Transform(matrix);
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