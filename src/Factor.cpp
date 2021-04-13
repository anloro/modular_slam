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


template <class T, class P>
class Factor
{
    // It specifies the prior between nodes
public:
    Factor(int from_node, int to_node, T mean, P noise);

private:
    // specify between which nodes
    int _from_node;
    int _to_node;
    // specify the mean
    T _mean;
    // specify noise model/uncertainty of the factor
    P _noise;
};

template <class T, class P>
Factor<T, P>::Factor(int from_node, int to_node, T mean, P noise)
{
    _from_node = from_node;
    _to_node = to_node;
    _mean = mean;
    _noise = noise;
}

int main()
{
    gtsam::Point3 t;
    t = gtsam::Point3(0.0, 0.0, 0.0);

    gtsam::Rot3 r;
    r = gtsam::Rot3(0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);

    gtsam::Pose3 new_pose;
    new_pose = gtsam::Pose3(r, t);

    // Test the Factor
    gtsam::Vector3 n;
    n = gtsam::Vector3(0.2, 0.2, 0.1);
    Factor<gtsam::Pose3, gtsam::Vector3> factor0(0, 1, new_pose, n);

    return 0;
}