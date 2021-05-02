/**
 * @file   WorldModel.cpp
 * @brief  WorldModel definitions.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

// my includes
#include "WorldModel.h"

#include <gtsam/sam/RangeFactor.h>
#include <gtsam/sam/BearingFactor.h>

// includes to create the custom graph
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>
// includes to the gtsam graph
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
// #include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <iostream>
#include <chrono>

using namespace gtsam;

WorldModel::WorldModel()
{
    // do we create the graph here?
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
// ------------- Entity creation definitions ---------------
// ---------------------------------------------------------
// void WorldModel::AddEntityLandMark(int nodeId, double x, double y, double z)
// {
//     LandMark newFrame = LandMark(x, y, z);
// }

// void WorldModel::AddEntityRefFrame(int nodeId, double x, double y, double z, double roll, double pitch, double yaw)
// {
//     RefFrame newFrame = RefFrame(x, y, z, roll, pitch, yaw);
// }

// Pose2 to Pose3 case 
// void WorldModel::AddEntityKeyFrame(int nodeId, double x, double y, double theta)
// {
//     // add new initial estimate of a node
//     Pose2 newPose2 = Pose2(x, y, theta);
//     Pose3 newPose = Pose3(newPose2);
//     // get the rotation as yaw pitch roll
//     Rot3 R = newPose.rotation();
//     Vector3 yawpitchroll = R.ypr();
//     double yaw = yawpitchroll[0];
//     double pitch = yawpitchroll[1];
//     double roll = yawpitchroll[2];
//     // TODO possibly add the transform to the sensor reference frame
//     KeyFrame newFrame = KeyFrame(x, y, 0, roll, pitch, yaw);
//     // std::pair<int, KeyFrame> newPair = std::pair(nodeId, newFrame);
//     _myKeyFrameMap.insert(std::make_pair(nodeId, newFrame));
//     AddInitialEstimate3ToGtsam(nodeId, x, y, 0, roll, pitch, yaw);
// }

// void WorldModel::AddEntityKeyFrame(int nodeId, double x, double y, double z, double roll, double pitch, double yaw)
// {
//     KeyFrame<int> newFrame = KeyFrame<int>(x, y, z, roll, pitch, yaw);
//     AddInitialEstimate3ToGtsam(nodeId, x, y, z, roll, pitch, yaw);
// }

// template <typename... Ts>
// void WorldModel::AddEntityKeyFrame(int nodeId, double x, double y, double z, double roll, double pitch, double yaw, Ts... data)
// {
//     KeyFrame<Ts...> newKeyFrame;
//     newKeyFrame = KeyFrame<Ts...>(x, y, z, roll, pitch, yaw, data...);
//     AddInitialEstimate3ToGtsam(nodeId, x, y, z, roll, pitch, yaw);
// }

// ---------------------------------------------------------
// ---------------- Interface with Gtsam -------------------
// ---------------------------------------------------------

// Add new initial estimate of a node in gtsam
void WorldModel::AddInitialEstimate3ToGtsam(int nodeId, double x, double y, double z, double roll, double pitch, double yaw)
{
    Rot3 newR = Rot3().Yaw(yaw).Pitch(pitch).Roll(roll);
    Point3 newP = Point3(x, y, z);
    Pose3 newPose = Pose3(newR, newP);
    _initialEstimate.insert(nodeId, newPose);
}

// ---------------------------------------------------------
// ------------- Factor creation definitions ---------------
// ---------------------------------------------------------

// Add new factor to gtsam posegraph in the range-only case
void WorldModel::AddRangeOnlyFactor(int fromNode, int toNode, double range, double sigmaRange)
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
void WorldModel::AddPoseFactor(int fromNode, int toNode, double x, double y, double theta, double sigmaX, double sigmaY, double sigmaTheta)
{
    auto noiseModel = noiseModel::Diagonal::Sigmas((Vector(6) << sigmaX, sigmaY, 0, 0, 0, sigmaTheta).finished());
    Pose2 newMean2 = Pose2(x, y, theta);
    Pose3 newMean = Pose3(newMean2);
    _graph.emplace_shared<BetweenFactor<Pose3>>(fromNode, toNode, newMean, noiseModel);
}

// Add new factor to gtsam posegraph in the 3D case
void WorldModel::AddPoseFactor(int fromNode, int toNode, double x, double y, double z, double roll, double pitch, double yaw, double sigmaX, double sigmaY, double sigmaZ, double sigmaRoll, double sigmaPitch, double sigmaYaw)
{
    auto noiseModel = noiseModel::Diagonal::Sigmas((Vector(6)<<sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw).finished());
    Rot3 newR = Rot3().Yaw(yaw).Pitch(pitch).Roll(roll);
    Point3 newP = Point3(x, y, z);
    Pose3 newMean = Pose3(newR, newP);
    _graph.emplace_shared<BetweenFactor<Pose3>>(fromNode, toNode, newMean, noiseModel);
}

void WorldModel::Optimize(){
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
    RefFrame globalRef = RefFrame(0, 0, 0, 0, 0, 0);
    myWorld.AddInitialEstimate3ToGtsam(0, 0, 0, 0, 0, 0, 0);

    // Add the Node 1
    KeyFrame node1 = KeyFrame<int>(2, 0, 0, 0, 0, 0);
    myWorld.AddEntity(1, node1);
    myWorld.AddInitialEstimate3ToGtsam(1, 2, 0, 0, 0, 0, 0);
    // Add Factor between Node 0 and Node 1
    myWorld.AddPoseFactor(0, 1, 2, 0, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);

    // Add the Node 2
    KeyFrame node2 = KeyFrame<int>(2, -1, 0, 0, 0, 0);
    myWorld.AddEntity(2, node2);
    myWorld.AddInitialEstimate3ToGtsam(2, -1, 0, 0, 0, 0, 0);
    // Add Factor between Node 1 and Node 2
    myWorld.AddPoseFactor(1, 2, 0, -1, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);

    // Add the Node 3
    KeyFrame node3 = KeyFrame<int>(4, -1, 0, 0, 0, 0);
    myWorld.AddEntity(3, node3);
    myWorld.AddInitialEstimate3ToGtsam(3, 4, -1, 0, 0, 0, 0);
    // Add Factor between Node 2 and Node 3
    myWorld.AddPoseFactor(2, 3, 2, 0, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);

    // Add the Node 4
    KeyFrame node4 = KeyFrame<int>(7, -2, 0, 0, 0, 0);
    myWorld.AddEntity(4, node4);
    myWorld.AddInitialEstimate3ToGtsam(4, 7, -2, 0, 0, 0, 0);
    // Add Factor between Node 3 and Node 4
    myWorld.AddPoseFactor(3, 4, 3, -1, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);

    // Add the Node 5
    KeyFrame node5 = KeyFrame<int>(7, 0, 0, 0, 0, 0);
    myWorld.AddEntity(5, node5);
    myWorld.AddInitialEstimate3ToGtsam(5, 7, 0, 0, 0, 0, 0);
    // Add Factor between Node 4 and Node 5
    myWorld.AddPoseFactor(4, 5, 0, 2, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);

    // Add the Node 6
    KeyFrame node6 = KeyFrame<int>(5, 3, 0, 0, 0, 0);
    myWorld.AddEntity(6, node6);
    myWorld.AddInitialEstimate3ToGtsam(6, 5, 3, 0, 0, 0, 0);
    // Add Factor between Node 5 and Node 6
    myWorld.AddPoseFactor(5, 6, -2, -3, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);

    // Add loop closure constraint
    myWorld.AddPoseFactor(6, 1, -3, -3, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);

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