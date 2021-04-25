/**
 * @file   WorldModel.cpp
 * @brief  Defines the Key-Frame Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

// my includes
#include "WorldModel.h"

// includes to create the custom graph
#include <opencv2/core/core.hpp>
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
    int initId = 1;
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
// ---------------- Interface with Gtsam -------------------
// ---------------------------------------------------------

void WorldModel::AddInitialEstimate3ToGtsam(int nodeId, double x, double y, double z, double roll, double pitch, double yaw)
{
    // add new initial estimate of a node in gtsam
    Rot3 newR = Rot3().Yaw(yaw).Pitch(pitch).Roll(roll);
    Point3 newP = Point3(x, y, z);
    Pose3 newPose = Pose3(newR, newP);
    _initialEstimate.insert(nodeId, newPose);
}

// ---------------------------------------------------------
// ------------- Entity creation definitions ---------------
// ---------------------------------------------------------
void WorldModel::AddEntityLandMark(int nodeId, double x, double y, double z)
{
    LandMark newFrame = LandMark(x, y, z);
}

void WorldModel::AddEntityRefFrame(int nodeId, double x, double y, double z, double roll, double pitch, double yaw)
{
    RefFrame newFrame = RefFrame(x, y, z, roll, pitch, yaw);
}

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

void WorldModel::AddFactor(int fromNode, int toNode, double x, double y, double theta, double sigmaX, double sigmaY, double sigmaTheta)
{
    // add new factor to gtsam posegraph in the 2D case
    auto noiseModel = noiseModel::Diagonal::Sigmas((Vector(6) << sigmaX, sigmaY, 0, 0, 0, sigmaTheta).finished());
    Pose2 newMean2 = Pose2(x, y, theta);
    Pose3 newMean = Pose3(newMean2);
    _graph.emplace_shared<BetweenFactor<Pose3>>(fromNode, toNode, newMean, noiseModel);
}

void WorldModel::AddFactor(int fromNode, int toNode, double x, double y, double z, double roll, double pitch, double yaw, double sigmaX, double sigmaY, double sigmaZ, double sigmaRoll, double sigmaPitch, double sigmaYaw)
{
    // add new factor to gtsam posegraph in the 2D case
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

    // // Test the WorldModel constructors for 2D
    // myWorld.AddFactor(1, 2, 2, 0, 0, 0.2, 0.2, 0.1);
    // myWorld.AddFactor(2, 3, 2, 0, M_PI_2, 0.2, 0.2, 0.1);
    // myWorld.AddFactor(3, 4, 2, 0, M_PI_2, 0.2, 0.2, 0.1);
    // myWorld.AddFactor(4, 5, 2, 0, M_PI_2, 0.2, 0.2, 0.1);
    // myWorld.AddFactor(5, 2, 2, 0, M_PI_2, 0.2, 0.2, 0.1);
    // // add the initial estimates
    // myWorld.AddEntity(1, 0.5, 0.0, 0.2);
    // myWorld.AddEntity(2, 2.3, 0.1, -0.2);
    // myWorld.AddEntity(3, 4.1, 0.1, M_PI_2);
    // myWorld.AddEntity(4, 4.0, 2.0, M_PI);
    // myWorld.AddEntity(5, 2.1, 2.1, -M_PI_2);

    // Test the WorldModel constructors for 3D
    // myWorld.AddFactor(1, 2, 2, 0, 0, 0, 0,      0, 0.2, 0.2, 0, 0, 0, 0.1);
    // myWorld.AddFactor(2, 3, 2, 0, 0, 0, 0, M_PI_2, 0.2, 0.2, 0, 0, 0, 0.1);
    // myWorld.AddFactor(3, 4, 2, 0, 0, 0, 0, M_PI_2, 0.2, 0.2, 0, 0, 0, 0.1);
    // myWorld.AddFactor(4, 5, 2, 0, 0, 0, 0, M_PI_2, 0.2, 0.2, 0, 0, 0, 0.1);
    // myWorld.AddFactor(5, 2, 2, 0, 0, 0, 0, M_PI_2, 0.2, 0.2, 0, 0, 0, 0.1);
    // // add the initial estimates
    // myWorld.AddEntity(1, 0.5, 0.0, 0, 0, 0, 0.2);
    // myWorld.AddEntity(2, 2.3, 0.1, 0, 0, 0, -0.2);
    // myWorld.AddEntity(3, 4.1, 0.1, 0, 0, 0, M_PI_2);
    // myWorld.AddEntity(4, 4.0, 2.0, 0, 0, 0, M_PI);
    // myWorld.AddEntity(5, 2.1, 2.1, 0, 0, 0, -M_PI_2);

    LandMark newLM = LandMark(0, 0, 0);
    std::tuple tLM = newLM.GetTranslationalVector();
    double xLM = std::get<0>(tLM);
    double yLM = std::get<1>(tLM);
    double zLM = std::get<2>(tLM);
    std::cout << "My LM translational vector: (" << xLM << ", " << yLM << ", " << zLM << ")" << std::endl;

    RefFrame newRF = RefFrame(0, 0, 0, 0, 0, 0);
    std::tuple tRF = newRF.GetTranslationalVector();
    double xRF = std::get<0>(tRF);
    double yRF = std::get<1>(tRF);
    double zRF = std::get<2>(tRF);
    std::cout << "My RF translational vector: (" << xRF << ", " << yRF << ", " << zRF << ")" << std::endl;
    std::tuple rRF = newRF.GetRotationalVector();
    double rollRF = std::get<0>(rRF);
    double pitchRF = std::get<1>(rRF);
    double yawRF = std::get<2>(rRF);
    std::cout << "My RF rotational vector: (" << rollRF << ", " << pitchRF << ", " << yawRF << ")" << std::endl;

    struct mystructure
    {
        double a;
        int b ;
        char c;
    };
    mystructure thisstruct {0, 23, 'a'};

    KeyFrame<mystructure> newKF = KeyFrame<mystructure>(0, 0, 0, 0, 0, 0, thisstruct);
    mystructure myObject = newKF.GetData<mystructure>();
    int myb = myObject.b;
    std::cout << "I stored this inside the KF: (" << myb << ")" << std::endl;

    std::tuple tKF = newKF.GetTranslationalVector();
    double xKF = std::get<0>(tKF);
    double yKF = std::get<1>(tKF);
    double zKF = std::get<2>(tKF);
    std::cout << "My KF translational vector: (" << xKF << ", " << yKF << ", " << zKF << ")" << std::endl;
    std::tuple rKF = newKF.GetRotationalVector();
    double rollKF = std::get<0>(rKF);
    double pitchKF = std::get<1>(rKF);
    double yawKF = std::get<2>(rKF);
    std::cout << "My KF rotational vector: (" << rollKF << ", " << pitchKF << ", " << yawKF << ")" << std::endl;

    // myWorld._myMap.insert(std::make_pair(1, newKF));
    // KeyFrame<mystructure> storedKF = boost::any_cast<KeyFrame<mystructure>>(myWorld._myMap[1]);

    myWorld.AddEntity(1, newKF);
    KeyFrame<mystructure> storedKF = myWorld.GetEntity<KeyFrame<mystructure>>(1);
    mystructure myStoredObject = storedKF.GetData<mystructure>();
    int mystoredb = myStoredObject.b;
    std::cout << "I stored this inside the entity map: (" << mystoredb << ")" << std::endl;

    // optimize
    // myWorld.Optimize();

    return 0;
}