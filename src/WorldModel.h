// my includes
#include "Entity.h"

// includes to create the custom graph
#include <opencv2/core/core.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
// includes to the gtsam graph
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
// #include <gtsam/geometry/Pose2.h>
// #include <gtsam/inference/Key.h>
// #include <gtsam/nonlinear/Marginals.h>

#include <iostream>
#include <chrono>

class WorldModel{
    public:
        WorldModel();
        // 2D case
        void AddEntity(int nodeId, double x, double y, double theta);
        void AddFactor(int fromNode, int toNode, double x, double y, double theta, double sigmaX, double sigmaY, double sigmaTheta);
        // 3D case
        void AddEntity(int nodeId, double x, double y, double z, double roll, double pitch, double yaw);
        void AddFactor(int fromNode, int toNode, double x, double y, double z, double roll, double pitch, double yaw, double sigmaX, double sigmaY, double sigmaZ, double sigmaRoll, double sigmaPitch, double sigmaYaw);
        // Optimization
        void Optimize();

    // private :
        // save the info of the custom pose graph
        // std::vector<Entity> _entities;
        // Factor _factors;

        // save the info for gtsam
        gtsam::NonlinearFactorGraph _graph;
        gtsam::Values _initialEstimate;
        gtsam::Values _result;
};