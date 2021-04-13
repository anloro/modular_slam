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
        // case 2D
        void AddEntity(int nodeId, double pose[3]);
        void AddFactor(int fromNode, int toNode, double mean[3], double noise[3]);
        void Optimize();

    // private :
        // save the info of the custom pose graph
        // std::vector<Entity> _entities;
        // Factor _factors;

        // save the info for gtsam
        gtsam::NonlinearFactorGraph _graph;
        gtsam::Values _initialEstimate;
        gtsam::GaussNewtonOptimizer _optimizer;
        gtsam::Values _result;
};