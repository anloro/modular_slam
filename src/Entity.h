
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

class Entity
{
public:
    Entity();
    Entity(int id, gtsam::Pose3 pose);
    Entity(int id, gtsam::Pose3 pose, int timestamp, cv::Mat image, cv::Mat depth);
    // ~Entity();

    struct raw_sensor_data
    {
        // unique timestamp
        int timestamp;
        // visual data
        cv::Mat image;
        cv::Mat depth; // depth information from the 2D image
        // lidar data
    };

private:
    gtsam::Pose3 _pose;
    int _id;
    raw_sensor_data _raw_sensor_data;
};
