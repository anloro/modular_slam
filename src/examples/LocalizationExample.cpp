/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file LocalizationExample.cpp
 * @brief Simple robot localization example, with three "GPS-like" measurements
 * @author Frank Dellaert
 */

/**
 * A simple 2D pose slam example with "GPS" measurements
 *  - The robot moves forward 2 meter each iteration
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - We have full odometry between pose
 *  - We have "GPS-like" measurements implemented with a custom factor
 */

// We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>

// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>

// As in OdometryExample.cpp, we use a BetweenFactor to model odometry measurements.
#include <gtsam/slam/BetweenFactor.h>

// We add all facors to a Nonlinear Factor Graph, as our factors are nonlinear.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// standard Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;

// Before we begin the example, we must create a custom unary factor to implement a
// "GPS-like" functionality. Because standard GPS measurements provide information
// only on the position, and not on the orientation, we cannot use a simple prior to
// properly model this measurement.
//
// The factor will be a unary factor, affect only a single system variable. It will
// also use a standard Gaussian noise model. Hence, we will derive our new factor from
// the NoiseModelFactor1.
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <gtsam/sam/RangeFactor.h>

class UnaryFactor : public NoiseModelFactor1<Pose3>
{
    // The factor will hold a measurement consisting of an (X,Y) location
    // We could this with a Point2 but here we just use two doubles
    double mx_, my_;

public:
    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<UnaryFactor> shared_ptr;

    // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
    UnaryFactor(Key j, double x, double y, const SharedNoiseModel &model) : NoiseModelFactor1<Pose3>(model, j), mx_(x), my_(y) {}

    ~UnaryFactor() override {}

    // Using the NoiseModelFactor1 base class there are two functions that must be overridden.
    // The first is the 'evaluateError' function. This function implements the desired measurement
    // function, returning a vector of errors when evaluated at the provided variable value. It
    // must also calculate the Jacobians for this measurement function, if requested.
    Vector evaluateError(const Pose3 &q,
                         boost::optional<Matrix &> H = boost::none) const override
    {
        // The measurement function for a GPS-like measurement is simple:
        // error_x = pose.x - measurement.x
        // error_y = pose.y - measurement.y
        // Consequently, the Jacobians are:
        // [ derror_x/dx  derror_x/dy  derror_x/dz derror_x/droll derror_x/dpitch derror_x/dyaw ] = [1 0 0 0 0 0]
        // [ derror_y/dx  derror_y/dy  derror_y/dz derror_y/droll derror_y/dpitch derror_y/dyaw ] = [0 1 0 0 0 0]
        if (H)
            (*H) = (Matrix(2, 6) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0).finished();
        return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
    }

    // The second is a 'clone' function that allows the factor to be copied. Under most
    // circumstances, the following code that employs the default copy constructor should
    // work fine.
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this)));
    }

    // Additionally, we encourage you the use of unit testing your custom factors,
    // (as all GTSAM factors are), in which you would need an equals and print, to satisfy the
    // GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these are not needed below.
}; // UnaryFactor

class RangeOnlyFactor : public NoiseModelFactor2<Pose3, Pose3>
{
    // The factor will hold a measurement consisting of an (X,Y) location
    // We could this with a Point2 but here we just use two doubles
    double mr_;

public:
    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<RangeOnlyFactor> shared_ptr;

    // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
    RangeOnlyFactor(Key j, Key k, double r, const SharedNoiseModel &model) : NoiseModelFactor2<Pose3, Pose3>(model, j, k), mr_(r) {}

    ~RangeOnlyFactor() override {}

    // Using the NoiseModelFactor2 base class there are two functions that must be overridden.
    // The first is the 'evaluateError' function. This function implements the desired measurement
    // function, returning a vector of errors when evaluated at the provided variable value. It
    // must also calculate the Jacobians for this measurement function, if requested.
    Vector evaluateError(const Pose3 &q, const Pose3 &p, boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none) const override
    {
        // The measurement function for a range-only measurement is simple:
        // error_r = pose.r - measurement.r
        // where r:
        // r = sqrt(dx²+dy²)
        // and dx, dy:
        // dx = x2 - x1; dy = y2 - y1;
        double dx, dy, r2;
        Vector e;
        // get the differentials
        dx = p.x() - q.x();
        dy = p.y() - q.y();
        // get the l2 norm
        r2 = sqrt(dx * dx + dy * dy);
        // Consequently, the Jacobians are:
        // [ derror/dx1  derror/dy1  derror/dz1 derror/droll1 derror/dpitch1 derror/dyaw1 ] = [-dx/r2 -dy/r2 0 0 0 0]
        // [ derror/dx2  derror/dy2  derror/dz2 derror/droll2 derror/dpitch2 derror/dyaw2 ] = [ dx/r2  dy/r2 0 0 0 0]
        if (H1)
            (*H1) = (Matrix(1, 6) << -dx / r2, -dy / r2, 0.0, 0.0, 0.0, 0.0).finished();
        if (H2)
            (*H2) = (Matrix(1, 6) << dx / r2, dy / r2, 0.0, 0.0, 0.0, 0.0).finished();
        e = (Vector(1) << abs(r2 - mr_)).finished();
        return e;
    }

    // The second is a 'clone' function that allows the factor to be copied. Under most
    // circumstances, the following code that employs the default copy constructor should
    // work fine.
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new RangeOnlyFactor(*this)));
    }

    // Additionally, we encourage you the use of unit testing your custom factors,
    // (as all GTSAM factors are), in which you would need an equals and print, to satisfy the
    // GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these are not needed below.
}; // RangeOnlyFactor

int main(int argc, char** argv) {
  // 1. Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;

  // Add a prior on the first pose, setting it to the origin
  Pose2 priorPose2 = Pose2(0, 0, 0);
  Pose3 priorPose = Pose3(priorPose2);
  auto priorNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.3, 0.3, 0.0, 0.0, 0.0, 0.1).finished());
  graph.addPrior(1, priorPose, priorNoise);

  // 2a. Add odometry factors
  // For simplicity, we will use the same noise model for each odometry factor
//   auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  auto odometryNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.2, 0.2, 0.0, 0.0, 0.0, 0.1).finished());
  // Create odometry (Between) factors between consecutive poses
  graph.emplace_shared<BetweenFactor<Pose3> >(1, 2, Pose3(Pose2(2.0, 0.0, 0.0)), odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose3> >(2, 3, Pose3(Pose2(2.0, 0.0, 0.0)), odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose3> >(3, 1, Pose3(Pose2(-4.0, 0.0, 0.0)), odometryNoise);

  // 2b. Add "GPS-like" measurements
  // We will use our custom UnaryFactor for this.
  auto unaryNoise =
      noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));  // 10cm std on x,y
//   graph.emplace_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise);
//   graph.emplace_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise);
//   graph.emplace_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise);

  // 2c. Add "range-only" measurements
  // We will use our custom UnaryFactor for this.
  auto rangeOnlyNoise =
      noiseModel::Diagonal::Sigmas(Vector1(0.1)); // 10cm std on r
  graph.emplace_shared<RangeFactor<Pose3, Pose3>>(1, 2, 2.0, rangeOnlyNoise);

  graph.print("\nFactor Graph:\n");  // print
  
  // 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initialEstimate;
  initialEstimate.insert(1, Pose3(Pose2(0.5, 0.0, 0.2)));
  initialEstimate.insert(2, Pose3(Pose2(2.3, 0.1, -0.2)));
  initialEstimate.insert(3, Pose3(Pose2(4.1, 0.1, 0.1)));
  initialEstimate.print("\nInitial Estimate:\n");  // print

  // 4. Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  GaussNewtonOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // 5. Calculate and print marginal covariances for all variables
  Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

  return 0;
}