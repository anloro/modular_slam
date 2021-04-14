
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
    double initPose[3] = {0, 0, 0};
    double initNoise[3] = {0.3, 0.3, 0.1};
    // Add a prior on the first pose, setting it to the origin
    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(initNoise[0], initNoise[1], initNoise[2]));
    _graph.addPrior(initId, Pose2(initPose[0], initPose[1], initPose[2]), priorNoise);
}

// ---------------------------------------------------------
// ------------------- 2D CASE -----------------------------
// ---------------------------------------------------------

void WorldModel::AddEntity(int nodeId, double pose[])
{
    // add new initial estimate of a node
    Pose2 newPose = Pose2(pose[0], pose[1], pose[2]);
    _initialEstimate.insert(nodeId, newPose);
}

void WorldModel::AddFactor(int fromNode, int toNode, double mean[], double noise_sigmas[])
{
    // add new factor to gtsam posegraph in the 2D case
    auto noise_model = noiseModel::Diagonal::Sigmas(Vector3(noise_sigmas[0], noise_sigmas[1], noise_sigmas[2]));
    Pose2 newMean = Pose2(mean[0], mean[1], mean[2]);
    _graph.emplace_shared<BetweenFactor<Pose2>>(fromNode, toNode, newMean, noise_model);
}

// ---------------------------------------------------------
// ------------------- 3D CASE -----------------------------
// ---------------------------------------------------------

// void WorldModel::AddEntity(int nodeId, double pose[6])
// {
//     // add new initial estimate of a node
//     Rot3 newR = Rot3::RzRyRx(pose[0], pose[1], pose[2]);
//     Point3 newP = Point3(pose[3], pose[4], pose[5]);
//     Pose3 newPose = Pose3(newR, newP);
//     _initialEstimate.insert(nodeId, newPose);
// }

// void WorldModel::AddFactor(int fromNode, int toNode, double mean[6], double noise_sigmas[6])
// {
//     // add new factor to gtsam posegraph in the 2D case
//     auto noise_model = noiseModel::Diagonal::Sigmas(Vector3(noise_sigmas[0], noise_sigmas[1], noise_sigmas[2]));
//     Pose2 newMean = Pose2(mean[0], mean[1], mean[2]);
//     _graph.emplace_shared<BetweenFactor<Pose2>>(fromNode, toNode, newMean, noise_model);
// }

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

    // Testing entity creation with SE3 pose.
    Point3 t;
    t = Point3(0.0, 0.0, 0.0);
    std::cout<< t << std::endl;
    
    Rot3 r;
    r = Rot3(0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);
    std::cout << r << std::endl;

    Pose3 new_pose;
    new_pose = Pose3(r, t);
    std::cout << new_pose << std::endl;

    // Test the WorldModel constructors
    WorldModel myWorld = WorldModel();
    double p[3] = {2, 0, 0};
    double n[3] = {0.2, 0.2, 0.1};
    myWorld.AddFactor(1, 2, p, n);
    p[2] = M_PI_2;
    myWorld.AddFactor(2, 3, p, n);
    myWorld.AddFactor(3, 4, p, n);
    myWorld.AddFactor(4, 5, p, n);
    myWorld.AddFactor(5, 2, p, n);

    // add the initial estimates
    double a[3] = {0.5, 0.0, 0.2};
    myWorld.AddEntity(1, a);
    a[0] = 2.3; 
    a[1] = 0.1; 
    a[2] = -0.2;
    myWorld.AddEntity(2, a);
    a[0] = 4.1;
    a[1] = 0.1;
    a[2] = M_PI_2;
    myWorld.AddEntity(3,a);
    a[0] = 4.0;
    a[1] = 2.0;
    a[2] = M_PI;
    myWorld.AddEntity(4, a);
    a[0] = 2.1;
    a[1] = 2.1;
    a[2] = -M_PI_2;
    myWorld.AddEntity(5, a);

    myWorld.Optimize();

    return 0;
}