
// my includes
#include "WorldModel.h"

// includes to create the custom graph
#include <opencv2/core/core.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
// includes to the gtsam graph
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// #include <gtsam/geometry/Pose2.h>
// #include <gtsam/inference/Key.h>
// #include <gtsam/nonlinear/GaussNewtonOptimizer.h>
// #include <gtsam/nonlinear/Marginals.h>

#include <iostream>
#include <chrono>

WorldModel::WorldModel(){
    // do we create the graph here?
}

void WorldModel::AddEntity()
{
    // add new entity
}

void WorldModel::AddFactor(int from_node, int to_node, std::array<double, 3> mean, std::array<double, 3> noise)
{
    // add new factor
}

int main()
{

    // Testing entity creation with SE3 pose.
    gtsam::Point3 t;
    t = gtsam::Point3(0.0, 0.0, 0.0);
    std::cout<< t << std::endl;
    
    gtsam::Rot3 r;
    r = gtsam::Rot3(0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);
    std::cout << r << std::endl;

    gtsam::Pose3 new_pose;
    new_pose = gtsam::Pose3(r, t);
    std::cout << new_pose << std::endl;

    // Test the Entity constructors
    Entity node0; 
    node0 = Entity();

    return 0;
}