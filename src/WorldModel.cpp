
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
// ------------------- Pose2 to Pose 3 CASE -----------------------------
// ---------------------------------------------------------
void WorldModel::AddEntity(int nodeId, double x, double y, double theta)
{
    // add new initial estimate of a node
    Pose2 newPose2 = Pose2(x, y, theta);
    Pose3 newPose = Pose3(newPose2);
    _initialEstimate.insert(nodeId, newPose);
}

void WorldModel::AddFactor(int fromNode, int toNode, double x, double y, double theta, double sigmaX, double sigmaY, double sigmaTheta)
{
    // add new factor to gtsam posegraph in the 2D case
    auto noiseModel = noiseModel::Diagonal::Sigmas((Vector(6) << sigmaX, sigmaY, 0, 0, 0, sigmaTheta).finished());
    Pose2 newMean2 = Pose2(x, y, theta);
    Pose3 newMean = Pose3(newMean2);
    _graph.emplace_shared<BetweenFactor<Pose3>>(fromNode, toNode, newMean, noiseModel);
}

// ---------------------------------------------------------
// ------------------- Pose3 CASE -----------------------------
// ---------------------------------------------------------
void WorldModel::AddEntity(int nodeId, double x, double y, double z, double roll, double pitch, double yaw)
{
    // add new initial estimate of a node
    Rot3 newR = Rot3().Yaw(yaw).Pitch(pitch).Roll(roll);
    Point3 newP = Point3(x, y, z);
    Pose3 newPose = Pose3(newR, newP);
    _initialEstimate.insert(nodeId, newPose);
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
    WorldModel myWorld = WorldModel();

    // // Test the WorldModel constructors for 2D
    myWorld.AddFactor(1, 2, 2, 0, 0, 0.2, 0.2, 0.1);
    myWorld.AddFactor(2, 3, 2, 0, M_PI_2, 0.2, 0.2, 0.1);
    myWorld.AddFactor(3, 4, 2, 0, M_PI_2, 0.2, 0.2, 0.1);
    myWorld.AddFactor(4, 5, 2, 0, M_PI_2, 0.2, 0.2, 0.1);
    myWorld.AddFactor(5, 2, 2, 0, M_PI_2, 0.2, 0.2, 0.1);
    // add the initial estimates
    myWorld.AddEntity(1, 0.5, 0.0, 0.2);
    myWorld.AddEntity(2, 2.3, 0.1, -0.2);
    myWorld.AddEntity(3, 4.1, 0.1, M_PI_2);
    myWorld.AddEntity(4, 4.0, 2.0, M_PI);
    myWorld.AddEntity(5, 2.1, 2.1, -M_PI_2);

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

    // optimize
    myWorld.Optimize();

    return 0;
}