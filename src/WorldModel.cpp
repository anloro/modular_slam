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


anloro::WorldModel::WorldModel()
{
    _plotterThread = new std::thread(&WorldModelPlotter::Spin, &_plotter);
}

// ---------------------------------------------------------
// --------- Utilities for Front-end interaction -----------
// ---------------------------------------------------------

// Map the input ID into our own internal IDs
int anloro::WorldModel::NodeInternalMapId(int id)
{
    // TO-DO: Add a time check to comapre between ids of different front-ends!
    
    // std::cout << "The input id is: " << id << " with a count of " << _frontEndToModular_node.count(id) << std::endl;


    // Check if the ID is already registered in the WorldModel
    if (_frontEndToModular_node.count(id) == 0)
    {
        // ID not found, increase node count
        // std::cout << "The current node id is: " << currentNodeId << std::endl;

        currentNodeId++;

        // std::cout << "And now is: " << currentNodeId << std::endl;

        // And add it
        _frontEndToModular_node.insert(std::make_pair(id, currentNodeId));
        _modularToFrontEnd_node.insert(std::make_pair(currentNodeId, id));
        return currentNodeId;
    }
    else
    {
        // ID found
        return _frontEndToModular_node[id];
    }
}

// Recover original ID from internal IDs
int anloro::WorldModel::GetNodeIdFromInternalMap(int id)
{
    // Check for the ID
    if (_modularToFrontEnd_node.count(id) == 0)
    {
        // ID not found!
        std::cout << "WARNING: ID " << id << " not found!" << std::endl;
        return -1;
    }
    else
    {
        // ID found
        return _modularToFrontEnd_node[id];
    }
}

// Map the input ID into our own internal IDs
int anloro::WorldModel::LandMarkInternalMapId(int id)
{
    // Check if the ID is already registered in the WorldModel
    if (_frontEndToModular_lm.count(id) == 0)
    {
        // ID not found, increase LandMark count
        currentLandMarkId++;
        // And add it
        _frontEndToModular_lm.insert(std::make_pair(id, currentLandMarkId));
        _modularToFrontEnd_lm.insert(std::make_pair(currentLandMarkId, id));
        return currentLandMarkId;
    }
    else
    {
        // ID found
        return _frontEndToModular_lm[id];
    }
}

// Recover original ID from internal IDs
int anloro::WorldModel::GetLandMarkIdFromInternalMap(int id)
{
    // Check for the ID
    if (_modularToFrontEnd_lm.count(id) == 0)
    {
        // ID not found!
        // std::cout << "WARNING: ID " << id << " not found!" << std::endl;
        return -1;
    }
    else
    {
        // ID found
        return _modularToFrontEnd_lm[id];
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


int main(int argc, char **argv)
{
    // Create the WorldModel
    WorldModel worldModel;
    // Create the UDP server
    UdpServer server = UdpServer();
    int recvlen;
    
    // Create the buffer
    void * buffer = calloc(1, sizeof(struct MsgUdp));

    // Loop
    for (;;) 
    {    
        // Receive data using the buffer
        recvlen = server.Receive(buffer);
        // Interpret the data using the message structure
        MsgUdp * msg = (struct MsgUdp *)buffer;

        // Process the message
        switch (msg->type)
        {
        // RefFrame case
        case 'a':
        {
            float x, y, z, pitch, yaw, roll;
            x = msg->element.refFrame.pose.x;
            y = msg->element.refFrame.pose.y;
            z = msg->element.refFrame.pose.z;
            roll = msg->element.refFrame.pose.roll;
            pitch = msg->element.refFrame.pose.pitch;
            yaw = msg->element.refFrame.pose.yaw;
            Transform transform(x, y, z, roll, pitch, yaw);
            RefFrame *frame = new RefFrame(transform);
            worldModel.AddRefFrameEntity(frame);
            break;
        }

        // KeyFrame case
        case 'b':
        {
            std::cout << "INFO: Received KeyFrame command." << std::endl;

            float x, y, z, pitch, yaw, roll;
            double timeStamp;
            int id;
            x = msg->element.keyFrame.pose.x;
            y = msg->element.keyFrame.pose.y;
            z = msg->element.keyFrame.pose.z;
            roll = msg->element.keyFrame.pose.roll;
            pitch = msg->element.keyFrame.pose.pitch;
            yaw = msg->element.keyFrame.pose.yaw;
            id = msg->element.keyFrame.id;
            timeStamp = msg->element.keyFrame.timeStamp;
            Transform transform(x, y, z, roll, pitch, yaw);
            int internalId;
            int dummyData = 0;
            internalId = worldModel.NodeInternalMapId(id);
            KeyFrame<int> *node = new KeyFrame<int>(timeStamp, transform, dummyData);
            worldModel.AddKeyFrameEntity(internalId, node);
            break;
        }

        // LandMark case
        case 'c':
        {
            std::cout << "INFO: Received LandMark command." << std::endl;

            float x, y, z, pitch, yaw, roll, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
            double timeStamp;
            int landMarkId;
            x = msg->element.landmark.pose.x;
            y = msg->element.landmark.pose.y;
            z = msg->element.landmark.pose.z;
            roll = msg->element.landmark.pose.roll;
            pitch = msg->element.landmark.pose.pitch;
            yaw = msg->element.landmark.pose.yaw;
            sigmaX = msg->element.landmark.unc.sigmaX;
            sigmaY = msg->element.landmark.unc.sigmaY;
            sigmaZ = msg->element.landmark.unc.sigmaZ;
            sigmaRoll = msg->element.landmark.unc.sigmaRoll;
            sigmaPitch = msg->element.landmark.unc.sigmaPitch;
            sigmaYaw = msg->element.landmark.unc.sigmaYaw;
            landMarkId = msg->element.landmark.landMarkId;
            Transform transform = Transform(x, y, z, roll, pitch, yaw);

            int nodeId = worldModel.currentNodeId;

            std::cout << "Landmark " << landMarkId << " seen from node " << nodeId << ": \n" << transform.ToMatrix4f() << std::endl;

            // Only consider a landMark if we have odometry
            if(nodeId > 0)
            {
                // Check if the LandMark wasn't previously seen
                if (worldModel._frontEndToModular_lm.count(landMarkId) == 0)
                {
                    // Get an internal ID for the new landmark
                    int internalLandMarkId = worldModel.LandMarkInternalMapId(landMarkId);

                    LandMark *landmark = new LandMark();
                    Transform iniStimate;
                    // Set the initial estimate for the landmark in absolute coordinates
                    std::cout << "Current state: \n"<< worldModel.currentState.ToMatrix4f() << std::endl;
                    std::cout << "LandMark relative pose: \n"<< transform.ToMatrix4f() << std::endl;
                    std::cout << "LandMark absolute pose: "<< std::endl;
                    iniStimate = worldModel.currentState * transform;
                    std::cout << "Current state * lmRelPose: \n"<< iniStimate.ToMatrix4f() << std::endl;

                    landmark->SetInitialEstimate(iniStimate);
                    // Add the node from where it was detected
                    landmark->AddNode(nodeId, transform,
                                    sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);

                    worldModel.AddLandMarkEntity(internalLandMarkId, landmark);
                    std::cout << "INFO: Created landmark with id "<< landMarkId << " percieved in node " << nodeId << "!" << std::endl;
                    std::cout << "INFO: With relative transform: \n"<< transform.ToMatrix4f() << std::endl;
                    std::cout << "INFO: With variances: " 
                            << sigmaX << " " << sigmaY << " " << sigmaZ << " " << sigmaRoll << " " << sigmaPitch << " " << sigmaYaw << std::endl;

                }else{
                    // Get the internal ID of the stored landmark
                    int internalLandMarkId = worldModel.LandMarkInternalMapId(landMarkId);
                    // And retrive its pointer from the worldmodel
                    LandMark *landmark = worldModel.GetLandMarkEntity(internalLandMarkId);

                    // Check if the landmark was seen from an already registered node
                    if(landmark->ExistsNode(nodeId)){
                        // The robot is probably not moving
                        // std::cout << "Node "<< nodeId << " already registered in landMark " << landMarkId << "!" << std::endl;
                    }else{
                        

                        std::cout << "INFO: Added node "<< nodeId << " to landMark " << landMarkId << "!" << std::endl;
                        std::cout << "INFO: With relative transform: \n"<< transform.ToMatrix4f() << std::endl;
                        std::cout << "INFO: With variances: " 
                                  << sigmaX << " " << sigmaY << " " << sigmaZ << " " << sigmaRoll << " " << sigmaPitch << " " << sigmaYaw << std::endl;

                        // Optimize after recognizing the same landmark in another node
                        if (nodeId - worldModel._lastOptimizationNodeId > 1)
                        {
                            landmark->AddNode(nodeId, transform,
                                        sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);
                            worldModel.Optimize();
                            worldModel._lastOptimizationNodeId = nodeId;
                        }else{
                            landmark->AddNode(nodeId, transform,
                                        sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);
                        }
                    }
                }
            }
            break;
        }

        // PoseFactor case
        case 'd':{
            std::cout << "INFO: Received PoseFactor command." << std::endl;

            float x, y, z, pitch, yaw, roll, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
            int fromNode, toNode;
            x = msg->element.poseFactor.pose.x;
            y = msg->element.poseFactor.pose.y;
            z = msg->element.poseFactor.pose.z;
            roll = msg->element.poseFactor.pose.roll;
            pitch = msg->element.poseFactor.pose.pitch;
            yaw = msg->element.poseFactor.pose.yaw;
            sigmaX = msg->element.poseFactor.unc.sigmaX;
            sigmaY = msg->element.poseFactor.unc.sigmaY;
            sigmaZ = msg->element.poseFactor.unc.sigmaZ;
            sigmaRoll = msg->element.poseFactor.unc.sigmaRoll;
            sigmaPitch = msg->element.poseFactor.unc.sigmaPitch;
            sigmaYaw = msg->element.poseFactor.unc.sigmaYaw;
            fromNode = msg->element.poseFactor.idFrom;
            toNode = msg->element.poseFactor.idTo;
            Transform transform = Transform(x, y, z, roll, pitch, yaw);

            int internalFrom, internalTo;
            internalFrom = worldModel.NodeInternalMapId(fromNode);
            internalTo = worldModel.NodeInternalMapId(toNode);

            // std::cout << "Pose constraint from id: " << fromNode << " with internal id " << internalFrom << std::endl;
            // std::cout << "To id: " << toNode << " with internal id " << internalTo << std::endl;

            PoseFactor *poseFactor = new PoseFactor(internalFrom, internalTo,
                                                    transform,
                                                    sigmaX, sigmaY, sigmaZ,
                                                    sigmaRoll, sigmaPitch, sigmaYaw);

            worldModel.AddPoseFactor(poseFactor);
            break;
        }

        // SaveData command case
        case 'e':
            std::cout << "INFO: Received SavePoses command." << std::endl;

            worldModel.SavePosesRaw("PosesRaw.txt");
            break;

        // Optimized poses request case
        case 'f':
        {
            std::cout << "INFO: Received GetOptimizedPoses command." << std::endl;

            // Get the optimized poses
            std::map<int, Eigen::Affine3f> optimizedPoses;
            optimizedPoses = worldModel.GetOptimizedPoses();
            
            // Send a message with the size of the graph
            // MsgUdp newMsg;
            // newMsg.type = 'f';

            // newMsg.size = optimizedPoses.size();
            // Check if the last id is not registered yet
            // if (worldModel.GetNodeIdFromInternalMap(optimizedPoses.end()->first) < 0){
            //     newMsg.size = optimizedPoses.size() - 1;
            // }else{
            //     newMsg.size = optimizedPoses.size();
            // }
            // std::cout << "INFO: Size sent: " << newMsg.size << std::endl;
            // server.Send(newMsg);

            // Map the optimized poses into the corresponding id used by the front-end
            int nodeId, frontEndId;
            // std::map<int, Eigen::Affine3f> mappedOptimizedPoses;
            typedef std::pair<int, Eigen::Affine3f> optimizedPose;

            // Iterate over the KeyFrame's map
            // std::cout << "INFO: Size before loop: " << optimizedPoses.size() << std::endl;

            // Send the graph
            MsgUdp newMsg;
            newMsg.type = 'f';
            // Check if the last id is not registered yet
            // if (worldModel.GetNodeIdFromInternalMap(optimizedPoses.end()->first) < 0){
            //     newMsg.size = optimizedPoses.size() - 1;
            // }else{
            //     newMsg.size = optimizedPoses.size();
            // }

            newMsg.size = optimizedPoses.size();
            std::cout << "INFO: Sending graph with size: " << newMsg.size << " the whole size is: " << optimizedPoses.size() << std::endl;

            KeyFrameData kf;
            float x, y, z, roll, pitch, yaw;
            int i = 0;
            for (std::map<int, Eigen::Affine3f>::const_iterator iter = optimizedPoses.begin(); iter != optimizedPoses.end(); ++iter)
            {
                // Get the information of each node
                nodeId = iter->first;
                frontEndId = worldModel.GetNodeIdFromInternalMap(nodeId);
                Transform t = Transform(iter->second);
                t.GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
                // Check if the ID corresponds to a node from this Front-End
                if (frontEndId > -1)
                {
                    // std::cout << "INFO: Sending KeyFrame: " << nodeId << std::endl;
                    // std::cout << "INFO: Sending KeyFrame mapped id: " << frontEndId << std::endl;
                    // newMsg.element.keyFrame.id = frontEndId;
                    // newMsg.element.keyFrame.pose.x = x;
                    // newMsg.element.keyFrame.pose.y = y;
                    // newMsg.element.keyFrame.pose.z = z;
                    // newMsg.element.keyFrame.pose.roll = roll;
                    // newMsg.element.keyFrame.pose.pitch = pitch;
                    // newMsg.element.keyFrame.pose.yaw = yaw;
                    // server.Send(newMsg);

                    kf.id = frontEndId;
                    kf.timeStamp = 0;
                    kf.pose.x = x;
                    kf.pose.y = y;
                    kf.pose.z = z;
                    kf.pose.roll = roll;
                    kf.pose.pitch = pitch;
                    kf.pose.yaw = yaw;

                    newMsg.element.graph[i] = kf;
                    i++;
                }
            }            
            server.Send(newMsg);
            break;
        }

        // Optimization trigger command case
        case 'g':
            std::cout << "INFO: Received Optimize command." << std::endl;

            worldModel.Optimize();
            break;

        default:
            break;
        }
    }
}