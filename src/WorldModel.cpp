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
    // Apply the odometry correction
    // Let's simplify
    // float newX = keyFrame->GetTransform().X() + odomCorrection.X();
    // float newY = keyFrame->GetTransform().Y() + odomCorrection.Y();
    // Transform correctedPose = keyFrame->GetTransform().Clone();
    // correctedPose.SetTranslationalVector(newX, newY, keyFrame->GetTransform().Z());
    Transform correctedPose = keyFrame->GetTransform() * odomCorrection;
    // Transform correctedPose = odomCorrection * keyFrame->GetTransform();

    keyFrame->SetTransform(correctedPose);

    _keyFramesMap.insert(KeyFramePair(id, keyFrame));
    currentState = keyFrame->GetTransform();
    // Add to the render window
    UpdateCompletePlot();
}

// Add a Landmark Entity to the graph
void anloro::WorldModel::AddLandMarkEntity(int id, LandMark *LandMark)
{
    string fullId = 'l' + std::to_string(id);
    std::cout << "INFO: Input LM id: " << fullId << std::endl;
    _landMarksMap.insert(LandMarkPair(fullId, LandMark));
}

// Get an existing LandMark pointer from the map
LandMark* anloro::WorldModel::GetLandMarkEntity(int id)
{
    string fullId = 'l' + std::to_string(id);
    return _landMarksMap[fullId];
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
    // If we are doing odometry correction then undo it before the new optimization
    // this is because we only do odometry correction in the absolute pose and 
    // not the constraints (This should be fixed)
    if(lastLoopId > 0)
    {
        UndoOdometryCorrection(lastLoopId, _keyFramesMap.rbegin()->first, odomCorrection.inverse().ToMatrix4f());
    }

    NonlinearFactorGraph graph;
    Values initialEstimate;
    Values optimizedPoses;

    // Add a prior on the first pose, setting it to the origin
    int initId = 0;
    Pose2 priorPose2 = Pose2(0, 0, 0);
    Pose3 priorPose = Pose3(priorPose2);
    auto priorNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
    graph.addPrior(initId, priorPose, priorNoise);

    Transform transform;
    int idFrom, idTo, NodeId;
    string landMarkId;
    // Add the Pose3 factors
    float sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
    for (std::map<int, PoseFactor *>::const_iterator iter = _poseFactorsMap.begin(); iter != _poseFactorsMap.end(); ++iter)
    {
        idFrom = iter->second->From();
        idTo = iter->second->To();
        transform = iter->second->GetTransform();

        // The last factor is set to one for the loop closure
        // if(iter->first == _poseFactorsMap.rbegin()->first)
        // {
        //     std::cout << "INFO: the one with var 1 is " << iter->first << std::endl;
        //     std::cout << "with id from " << idFrom << " to " << idTo << std::endl;
        //     sigmaX = 1;
        //     sigmaY = 1;
        //     sigmaZ = 1;
        //     sigmaRoll = 1;
        //     sigmaPitch = 1;
        //     sigmaYaw = 1;
        //     iter->second->SetEulerVariances(sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);

        // }else{
        //     iter->second->GetEulerVariances(sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);
        // }
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
    for (std::map<string, LandMark*>::const_iterator iteri = _landMarksMap.begin(); iteri != _landMarksMap.end(); ++iteri)
    {
        landMarkId = iteri->first;
        int landMarkIdInt = landMarkId[1] - '0'; // get rid off the offset for conversion
        Symbol landMarkKey(landMarkId[0], landMarkIdInt);
        // Symbol landMarkKey(landMarkId);
        // std::cout << "INFO: The generated LM symbol: " << landMarkKey << std::endl;
        // Symbol landMarkKey('l', landMarkId);
        

        LandMark::RelatedNodesMap relatedNodes = iteri->second->GetRelatedNodes();
        // Add the constraints to the related nodes
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

            auto measurementNoise = noiseModel::Diagonal::Variances((Vector(6) << sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw).finished());
            // auto measurementNoise = noiseModel::Diagonal::Variances((Vector(3) << sigmaX, sigmaY, sigmaZ).finished());
            // auto measurementNoise = noiseModel::Diagonal::Variances((Vector(2) << 0.1, 0.1).finished());
            // auto measurementNoise = noiseModel::Diagonal::Variances((Vector(3) << 0.00001, 0.00001, 0.00001).finished());
            // auto measurementNoise = noiseModel::Diagonal::Variances((Vector(6) << 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001).finished());
            
            // Add transform as a Pose3 BetweenFactor
            Pose3 newMean = Pose3(transform.ToMatrix4f().cast<double>()); // Pose3 needs a double datatype
            graph.emplace_shared<BetweenFactor<Pose3>>(nodeId, landMarkKey, newMean, measurementNoise);

            // Add transform as a bearing range factor
            // Point3 landmark(transform.X(), transform.Y(), 0);
            // Pose3 p;
            // graph.add(BearingRangeFactor<Pose3, Point3>(nodeId, landMarkKey, p.bearing(landmark), p.range(landmark), measurementNoise));

            // Point2
            // Point2 landmark(transform.X(), transform.Y());
            // Pose2 p;
            // graph.add(BearingRangeFactor<Pose3, Point2>(nodeId, landMarkKey, p.bearing(landmark), p.range(landmark), measurementNoise));
        }

        // As a Point2
        // float xAbs, yAbs;
        // iteri->second->GetInitialEstimate().GetTranslationalVector(xAbs, yAbs, dummyVar);
        // Point2 pLandMark_abs_coord = Point2(xAbs, yAbs);
        // initialEstimate.insert(landMarkKey, pLandMark_abs_coord);

        // As a Point3
        // float xAbs, yAbs, zAbs;
        // iteri->second->GetInitialEstimate().GetTranslationalAndEulerAngles(xAbs, yAbs, zAbs, dummyVar, dummyVar, dummyVar);
        // Point3 pLandMark_abs_coord = Point3(xAbs, yAbs, zAbs);
        // initialEstimate.insert(landMarkKey, pLandMark_abs_coord);

        // As a Pose3
        float xAbs, yAbs, zAbs;
        Pose3 LandMarkPose3 = Pose3(iteri->second->GetInitialEstimate().ToMatrix4f().cast<double>());
        initialEstimate.insert(landMarkKey, LandMarkPose3);
    }

    std::cout << "-------------------------------------------" << std::endl;
    // std::cout << "The initial graph is: " << std::endl;
    // graph.print();
    // initialEstimate.print();

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

    // optimizer.values().print("After the optimization: \n *** \n");
    
    // Save the last odometry pose to compute the odometry correction
    Transform lastOdomPose = _keyFramesMap.rbegin()->second->GetTransform();
    int lastOdomPoseID = _keyFramesMap.rbegin()->first;

    Symbol graphKey, worldKey;
    // Update the World Model keyframes with the optimized poses.
    for (std::pair<Values::const_iterator, std::map<int, KeyFrame<int> *>::const_iterator> iter(optimizer.values().begin(), _keyFramesMap.begin());
         iter.first != optimizer.values().end() && iter.second != _keyFramesMap.end();
         ++iter.first, ++iter.second)
    {
        // Check if the iterator is in the same node.
        graphKey = (int)iter.first->key;
        worldKey = iter.second->first;
        // std::cout << "GraphKey: " << iter.first->key << " worldKey: " << iter.second->first << std::endl;
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
            std::cout << "ERROR: The World model KeyFrame update is failing!" << std::endl;
        }
    }

    // Update the World Model landmark estimate with the optimized poses.
    for (Values::const_iterator iterV = optimizer.values().begin(); iterV != optimizer.values().end(); ++iterV)
    {
        Symbol valueKey = iterV->key;
        for (std::map<string, LandMark *>::const_iterator iterL = _landMarksMap.begin(); iterL != _landMarksMap.end(); ++iterL)
        {
            // Check if the iterator is in the same node.
            landMarkId = iterL->first;
            int landMarkIdInt = landMarkId[1] - '0'; // get rid off the offset for conversion
            Symbol lmKey(landMarkId[0], landMarkIdInt);

            // std::cout << "INFO: Result key: " << valueKey << " LM: " << lmKey << std::endl;
            if (valueKey == lmKey)
            {
                // Get the optimized poses from the optimizer
                // Point2
                // Point2 optimizedPose = iterV->value.cast<Point2>();
                // Transform transform = Transform(optimizedPose[0], optimizedPose[1], 0, 0, 0, 0);

                // Point3
                // Point3 optimizedPose = iterV->value.cast<Point3>();
                // Transform transform = Transform(optimizedPose[0], optimizedPose[1], optimizedPose[2], 0, 0, 0);

                // Pose3
                Pose3 optimizedPose = iterV->value.cast<Pose3>();
                Eigen::Matrix4f matrix = optimizedPose.matrix().cast<float>();
                Transform transform = Transform(matrix);

                // Update the World Model with the optimized poses
                iterL->second->SetInitialEstimate(transform);
                std::cout << "INFO: The LandMark " << lmKey << " is updated with " << valueKey << " with transform:\n" << transform.ToMatrix4f() << std::endl;
            }
        }
    }

    Transform lastOptimizedPose = _keyFramesMap.rbegin()->second->GetTransform();
    int lastOptimizedPoseID = _keyFramesMap.rbegin()->first;

    // Let's simplify the computations
    // lastOptimizedPose.SetEulerAngles(0,0,0);
    // lastOdomPose.SetEulerAngles(0,0,0);
    // float optX, optY, optZ, odomX, odomY, odomZ;
    // lastOptimizedPose.GetTranslationalVector(optX, optY, optZ);
    // lastOdomPose.GetTranslationalVector(odomX, odomY, odomZ);
    // odomCorrection.SetTranslationalVector(optX - odomX, optY - odomY, 0);
    // odomCorrection = Transform(lastOdomPose.inverse().ToMatrix4f() * lastOptimizedPose.ToMatrix4f());

    std::cout << "INFO: lastOdomPose.inverse() * lastOptimizedPose: \n" << odomCorrection.ToMatrix4f() << std::endl;
    // lastLoopId = _keyFramesMap.rbegin()->first;

    std::cout << "INFO: Odom correction: \n" << odomCorrection.ToMatrix4f() << std::endl;
    std::cout << "INFO: LastOdomPose: " << lastOdomPoseID << "\n" << lastOdomPose.ToMatrix4f() << std::endl;
    std::cout << "INFO: lastOptimizedPose: " << lastOptimizedPoseID << "\n" << lastOptimizedPose.ToMatrix4f() << std::endl;

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

void anloro::WorldModel::UndoOdometryCorrection(int lastLoopId, int currentLoopId, Eigen::Matrix4f uncorrection)
{
    // int truid = InternalMapId(lastLoopId);
    Transform transform, newTransform;
    Eigen::Matrix4f uncorrected;

    // Iterate over the KeyFrame's map
    // for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter->first != lastLoopId; ++iter)
    // {
    //     transform = iter->second->GetTransform();
    //     uncorrected = transform.ToMatrix4f() * uncorrection;
    //     newTransform = Transform(uncorrected);

    //     iter->second->SetTransform(newTransform);
    // }

    // std::cout << "INFO: The uncorrection used is \n" << uncorrection << std::endl; 


    for (int i = lastLoopId; i < currentLoopId; i++)
    {
        transform = _keyFramesMap.at(i)->GetTransform();
        uncorrected = transform.ToMatrix4f() * uncorrection;
        newTransform = Transform(uncorrected);

        // std::cout << "INFO: Node " << i << " uncorrected from \n" << transform.ToMatrix4f() << "\nto \n" << newTransform.ToMatrix4f() << std::endl; 

        _keyFramesMap.at(i)->SetTransform(newTransform);
    }
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
    // Use the default name
    SavePosesRaw("rawposes.txt");
}

void anloro::WorldModel::SavePosesRaw(std::string name)
{
    // Create and open a text file
    std::ofstream outputFile(name);

    double timeStamp;
    int nodeId;
    float x, y, z, roll, pitch, yaw;
    Transform transform;

    // Add a line to identify each element
    outputFile << "# timestamp x y z roll pitch yaw id" << std::endl;

    // Iterate over the KeyFrame's map
    for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter != _keyFramesMap.end(); ++iter)
    {
        // Get the information of each node
        timeStamp = iter->second->GetTimeStamp();
        nodeId = iter->first;
        transform = iter->second->GetTransform();
        transform.GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);

        // Write to the file
        outputFile << timeStamp << " " << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << " " << nodeId << std::endl;
    }


    // Save the constraints
    int idFrom, idTo;
    outputFile << "\n# (constraints) idFrom idTo" << std::endl;
    // Iterate over the constraints map
    for (std::map<int, PoseFactor *>::const_iterator iter = _poseFactorsMap.begin(); iter != _poseFactorsMap.end(); ++iter)
    {
        idFrom = iter->second->From();
        idTo = iter->second->To();

        // Write to the file
        outputFile << idFrom << " " << idTo << std::endl;
    }

    // Save the landmarks
    string landMarkId;
    int lmId;
    outputFile << "\n# (landmarks) x y z roll pitch yaw id" << std::endl;
    // Iterate over the landmark map
    for (std::map<string, LandMark *>::const_iterator iter = _landMarksMap.begin(); iter != _landMarksMap.end(); ++iter)
    {
        // Get the information of each landmark
        landMarkId = iter->first;
        lmId = landMarkId[1] - '0'; // get rid off the offset for conversion
        transform = iter->second->GetInitialEstimate();
        transform.GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);

        // Write to the file
        outputFile << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << " " << lmId << std::endl;
    }

    // Add the constraints to the related nodes
    outputFile << "\n# (landmarks constraints) lmId relatedNodeId0 relatedNodeId1 ..." << std::endl;
    // Iterate over the landmark map
    for (std::map<string, LandMark *>::const_iterator iter = _landMarksMap.begin(); iter != _landMarksMap.end(); ++iter)
    {
        // Get the information of each landmark
        landMarkId = iter->first;
        lmId = landMarkId[1] - '0'; // get rid off the offset for conversion
        outputFile << lmId << " ";

        // Get the related nodes to the landmark
        LandMark::RelatedNodesMap relatedNodes = iter->second->GetRelatedNodes();
        for (LandMark::RelatedNodesMap::const_iterator iterj = relatedNodes.begin(); iterj != relatedNodes.end(); ++iterj)
        {
            nodeId = iterj->first;
            // Write to the file
            outputFile << nodeId << " ";
        }

        outputFile << std::endl;
    }

    std::cout << "Pose-graph information saved as: " << name << std::endl;

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