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

anloro::WorldModel::WorldModel()
{
    int initId = 0;
    // 2D CASE
    // Add a prior on the first pose, setting it to the origin
    Pose2 priorPose2 = Pose2(0, 0, 0);
    Pose3 priorPose = Pose3(priorPose2);
    // auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));

    // 3D CASE
    // Add a prior on the first pose, setting it to the origin
    auto priorNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.3, 0.3, 0.0, 0.0, 0.0, 0.1).finished());
    // Rot3 priorR = Rot3::RzRyRx(0, 0, 0);
    // Point3 priorP = Point3(0, 0, 0);
    // Pose3 priorPose = Pose3(priorR, priorP);

    _graph.addPrior(initId, priorPose, priorNoise);
}

// ---------------------------------------------------------
// --------- Utilities for Front-end interaction -----------
// ---------------------------------------------------------

// Map the input ID into our own internal IDs
int anloro::WorldModel::InternalMapId(int id)
{
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
void anloro::WorldModel::AddRefFrameEntity(RefFrame *refframe)
{
    static int refFrameID = 0; // initialized only once across all calls
    _refFramesMap.insert(RefFramePair(refFrameID, refframe));
    refFrameID++;
}

// Add a Key-Frame Entity to the graph
void anloro::WorldModel::AddKeyFrameEntity(int id, KeyFrame<int> *keyframe)
{
    int internalId;
    internalId = InternalMapId(id);
    _keyFramesMap.insert(KeyFramePair(internalId, keyframe));
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

// Add new initial estimate of a node in gtsam
void anloro::WorldModel::AddInitialEstimate3ToGtsam(int id, double x, double y, double z, double roll, double pitch, double yaw)
{
    int internalId;
    internalId = InternalMapId(id);
    Rot3 newR = Rot3().Yaw(yaw).Pitch(pitch).Roll(roll);
    Point3 newP = Point3(x, y, z);
    Pose3 newPose = Pose3(newR, newP);
    _initialEstimate.insert(internalId, newPose);
}

// Add new factor to gtsam posegraph in the range-only case
void anloro::WorldModel::AddRangeOnlyFactor(int fromNode, int toNode, double range, double sigmaRange)
{
    auto rangeOnlyNoise = noiseModel::Diagonal::Sigmas(Vector1(sigmaRange));
    _graph.emplace_shared<RangeFactor<Pose3, Pose3>>(fromNode, toNode, range, rangeOnlyNoise);
}

// Add new factor to gtsam posegraph in the bearing-only case
// void WorldModel::AddBearingOnlyFactor(int fromNode, int toNode, double bearing, double sigmaBearing)
// {
//     auto bearingOnlyNoise = noiseModel::Diagonal::Sigmas(Vector1(sigmaBearing)); 
//     _graph.emplace_shared<BearingFactor<Pose3, Pose3>>(fromNode, toNode, bearing, bearingOnlyNoise);
// }

// Add new factor to gtsam posegraph in the 2D case
void anloro::WorldModel::AddPoseFactor(int fromNode, int toNode, double x, double y, double theta, double sigmaX, double sigmaY, double sigmaTheta)
{
    auto noiseModel = noiseModel::Diagonal::Sigmas((Vector(6) << sigmaX, sigmaY, 0, 0, 0, sigmaTheta).finished());
    Pose2 newMean2 = Pose2(x, y, theta);
    Pose3 newMean = Pose3(newMean2);
    _graph.emplace_shared<BetweenFactor<Pose3>>(fromNode, toNode, newMean, noiseModel);
}

// Add new factor to gtsam posegraph in the 3D case
// void anloro::WorldModel::AddPoseFactor(int fromNode, int toNode, double x, double y, double z, double roll, double pitch, double yaw, double sigmaX, double sigmaY, double sigmaZ, double sigmaRoll, double sigmaPitch, double sigmaYaw)
// {
//     int internalFrom, internalTo;
//     internalFrom = InternalMapId(fromNode);
//     internalTo = InternalMapId(toNode);
//     auto noiseModel = noiseModel::Diagonal::Sigmas((Vector(6) << sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw).finished());
//     Rot3 newR = Rot3().Yaw(yaw).Pitch(pitch).Roll(roll);
//     Point3 newP = Point3(x, y, z);
//     Pose3 newMean = Pose3(newR, newP);
//     _graph.emplace_shared<BetweenFactor<Pose3>>(internalFrom, internalTo, newMean, noiseModel);
// }

void anloro::WorldModel::Optimize()
{
    // Let's debug. Check the node ids
    // for (std::map<int, boost::any>::iterator it = _myMap.begin(); it != _myMap.end(); ++it)
    // {
    //     std::cout << "Key: " << it->first << std::endl;
    //     // This getentity has to be changed
    //     // KeyFrameALR<int> kf = GetEntity<KeyFrameALR<int>>(it->first);
    //     // std::cout << "Value: " << it->second << std::endl;
    // }
    _graph.print();

    GaussNewtonParams parameters;
    GaussNewtonOptimizer optimizer(_graph, _initialEstimate, parameters);
    Values _result = optimizer.optimize();

    // This is for testing
    _result.print("Final Result:\n");
    // 5. Calculate and print marginal covariances for all variables
    std::cout.precision(3);
    Marginals marginals(_graph, _result);
    std::cout << "x1 covariance:\n"
              << marginals.marginalCovariance(1) << std::endl;
    std::cout << "x2 covariance:\n"
              << marginals.marginalCovariance(2) << std::endl;
    std::cout << "x3 covariance:\n"
              << marginals.marginalCovariance(3) << std::endl;
    std::cout << "x4 covariance:\n"
              << marginals.marginalCovariance(4) << std::endl;
    std::cout << "x5 covariance:\n"
              << marginals.marginalCovariance(5) << std::endl;
}

int main()
{
    WorldModel myWorld;

    // --------------------------------------------------------------
    // Graph configuration: -----------------------------------------
    // --------------------------------------------------------------
    //                                       [#]
    //                                    (5, 3, 0)
    //                                    [Node 6]
    //     [#]              [#]                             [#]
    //  (0, 0, 0)        (2, 0, 0)                       (7, 0, 0)
    //  [Node 0]          [Node 1]                        [Node 5]
    //  [Global ref.]
    //                      [#]             [#]             [#]
    //                   (2, -1, 0)      (4, -1, 0)      (7, -2, 0)
    //                    [Node 2]        [Node 3]        [Node 4]
    // --------------------------------------------------------------
    // --------------------------------------------------------------

    // Add the global reference frame
    RefFrame* globalRef = new RefFrame(0, 0, 0, 0, 0, 0);
    myWorld.AddRefFrameEntity(globalRef);
    myWorld.AddInitialEstimate3ToGtsam(0, 0, 0, 0, 0, 0, 0);

    // Add the Node 1
    int dummyData = 0;
    KeyFrame<int>* node1 = new KeyFrame<int>(2, 0, 0, 0, 0, 0, dummyData);
    myWorld.AddKeyFrameEntity(1, node1);
    myWorld.AddInitialEstimate3ToGtsam(1, 2, 0, 0, 0, 0, 0);
    // Add Factor between Node 0 and Node 1
    PoseFactor* poseFactor1 = new PoseFactor(0, 1, 2, 0, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);
    myWorld.AddPoseFactor(poseFactor1);

    // Add the Node 2
    KeyFrame<int> node2 = KeyFrame<int>(2, -1, 0, 0, 0, 0, dummyData);
    myWorld.AddEntity(2, node2);
    myWorld.AddInitialEstimate3ToGtsam(2, -1, 0, 0, 0, 0, 0);
    // Add Factor between Node 1 and Node 2
    PoseFactor* poseFactor2 = new PoseFactor(1, 2, 0, -1, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);
    myWorld.AddPoseFactor(poseFactor2);

    // Add the Node 3
    KeyFrame<int> node3 = KeyFrame<int>(4, -1, 0, 0, 0, 0, dummyData);
    myWorld.AddEntity(3, node3);
    myWorld.AddInitialEstimate3ToGtsam(3, 4, -1, 0, 0, 0, 0);
    // Add Factor between Node 2 and Node 3
    PoseFactor* poseFactor3 = new PoseFactor(2, 3, 2, 0, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);
    myWorld.AddPoseFactor(poseFactor3);

    // Add the Node 4
    KeyFrame<int> node4 = KeyFrame<int>(7, -2, 0, 0, 0, 0, dummyData);
    myWorld.AddEntity(4, node4);
    myWorld.AddInitialEstimate3ToGtsam(4, 7, -2, 0, 0, 0, 0);
    // Add Factor between Node 3 and Node 4
    PoseFactor* poseFactor4 = new PoseFactor(3, 4, 3, -1, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);
    myWorld.AddPoseFactor(poseFactor4);

    // Add the Node 5
    KeyFrame<int> node5 = KeyFrame<int>(7, 0, 0, 0, 0, 0, dummyData);
    myWorld.AddEntity(5, node5);
    myWorld.AddInitialEstimate3ToGtsam(5, 7, 0, 0, 0, 0, 0);
    // Add Factor between Node 4 and Node 5
    PoseFactor* poseFactor5 = new PoseFactor(4, 5, 0, 2, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);
    myWorld.AddPoseFactor(poseFactor5);

    // Add the Node 6
    KeyFrame<int> node6 = KeyFrame<int>(5, 3, 0, 0, 0, 0, dummyData);
    myWorld.AddEntity(6, node6);
    myWorld.AddInitialEstimate3ToGtsam(6, 5, 3, 0, 0, 0, 0);
    // Add Factor between Node 5 and Node 6
    PoseFactor* poseFactor6 = new PoseFactor(5, 6, -2, -3, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);
    myWorld.AddPoseFactor(poseFactor6);

    // Add loop closure constraint
    PoseFactor* poseFactor7 = new PoseFactor(6, 1, -3, -3, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);
    myWorld.AddPoseFactor(poseFactor7);

    // LandMark newLM = LandMark(0, 0, 0);
    // std::tuple tLM = newLM.GetTranslationalVector();
    // double xLM = std::get<0>(tLM);
    // double yLM = std::get<1>(tLM);
    // double zLM = std::get<2>(tLM);
    // std::cout << "My LM translational vector: (" << xLM << ", " << yLM << ", " << zLM << ")" << std::endl;

    // RefFrame newRF = RefFrame(0, 0, 0, 0, 0, 0);
    // std::tuple tRF = newRF.GetTranslationalVector();
    // double xRF = std::get<0>(tRF);
    // double yRF = std::get<1>(tRF);
    // double zRF = std::get<2>(tRF);
    // std::cout << "My RF translational vector: (" << xRF << ", " << yRF << ", " << zRF << ")" << std::endl;
    // std::tuple rRF = newRF.GetRotationalVector();
    // double rollRF = std::get<0>(rRF);
    // double pitchRF = std::get<1>(rRF);
    // double yawRF = std::get<2>(rRF);
    // std::cout << "My RF rotational vector: (" << rollRF << ", " << pitchRF << ", " << yawRF << ")" << std::endl;

    // struct mystructure
    // {
    //     double a;
    //     int b ;
    //     char c;
    // };
    // mystructure thisstruct {0, 23, 'a'};

    // KeyFrame<mystructure> newKF = KeyFrame<mystructure>(0, 0, 0, 0, 0, 0, thisstruct);
    // mystructure myObject = newKF.GetData<mystructure>();
    // int myb = myObject.b;
    // std::cout << "I stored this inside the KF: (" << myb << ")" << std::endl;

    // std::tuple tKF = newKF.GetTranslationalVector();
    // double xKF = std::get<0>(tKF);
    // double yKF = std::get<1>(tKF);
    // double zKF = std::get<2>(tKF);
    // std::cout << "My KF translational vector: (" << xKF << ", " << yKF << ", " << zKF << ")" << std::endl;
    // std::tuple rKF = newKF.GetRotationalVector();
    // double rollKF = std::get<0>(rKF);
    // double pitchKF = std::get<1>(rKF);
    // double yawKF = std::get<2>(rKF);
    // std::cout << "My KF rotational vector: (" << rollKF << ", " << pitchKF << ", " << yawKF << ")" << std::endl;

    // myWorld.AddEntity(1, newKF);
    // KeyFrame<mystructure> storedKF = myWorld.GetEntity<KeyFrame<mystructure>>(1);
    // mystructure myStoredObject = storedKF.GetData<mystructure>();
    // int mystoredb = myStoredObject.b;
    // std::cout << "I stored this inside the entity map: (" << mystoredb << ")" << std::endl;

    // Optimize
    myWorld.Optimize();

    return 0;
}