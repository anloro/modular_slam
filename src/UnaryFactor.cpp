#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>

using namespace gtsam;

class UnaryFactor : public NoiseModelFactor1<Pose2>
{
    // The factor will hold a measurement consisting of an (X,Y) location
    // We could this with a Point2 but here we just use two doubles
    double mx_, my_;

public:
    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<UnaryFactor> shared_ptr;

    // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
    UnaryFactor(Key j, double x, double y, const SharedNoiseModel &model) : NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

    ~UnaryFactor() override {}

    // Using the NoiseModelFactor1 base class there are two functions that must be overridden.
    // The first is the 'evaluateError' function. This function implements the desired measurement
    // function, returning a vector of errors when evaluated at the provided variable value. It
    // must also calculate the Jacobians for this measurement function, if requested.
    Vector evaluateError(const Pose2 &q,
                         boost::optional<Matrix &> H = boost::none) const override
    {
        // The measurement function for a GPS-like measurement is simple:
        // error_x = pose.x - measurement.x
        // error_y = pose.y - measurement.y
        // Consequently, the Jacobians are:
        // [ derror_x/dx  derror_x/dy  derror_x/dtheta ] = [1 0 0]
        // [ derror_y/dx  derror_y/dy  derror_y/dtheta ] = [0 1 0]
        if (H)
            (*H) = (Matrix(2, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0).finished();
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

class RangeOnlyFactor : public NoiseModelFactor2<Pose2, Pose2>
{
    // The factor will hold a measurement consisting of an (X,Y) location
    // We could this with a Point2 but here we just use two doubles
    double mr_;

public:
    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<RangeOnlyFactor> shared_ptr;

    // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
    RangeOnlyFactor(Key j, Key k, double r, const SharedNoiseModel &model) : NoiseModelFactor2<Pose2, Pose2>(model, j, k), mr_(r) {}

    ~RangeOnlyFactor() override {}

    // Using the NoiseModelFactor2 base class there are two functions that must be overridden.
    // The first is the 'evaluateError' function. This function implements the desired measurement
    // function, returning a vector of errors when evaluated at the provided variable value. It
    // must also calculate the Jacobians for this measurement function, if requested.
    Vector evaluateError(const Pose2 &q, const Pose2 &p, boost::optional<Matrix &> H1 =
                    boost::none, boost::optional<Matrix &> H2 = boost::none) const override
    {
        // The measurement function for a range-only measurement is simple:
        // error_r = pose.r - measurement.r
        // where r:
        // r = sqrt(dx²+dy²)
        // and dx, dy:
        // dx = x2 - x1; dy = y2 - y1; 
        double dx, dy, r2;
        // get the differentials
        dx = p.x() - q.x();
        dy = p.y() - q.y();
        // get the l2 norm
        r2 = sqrt(dx*dx + dy*dy);
        // Consequently, the Jacobians are:
        // [ derror/dx1  derror/dy1  derror/dtheta1 ] = [-dx/r2 -dy/r2 0]
        // [ derror/dx2  derror/dy2  derror/dtheta2 ] = [ dx/r2  dy/r2 0]
        if (H1)
            (*H1) = (Matrix(1, 3) << -dx / r2, -dy / r2, 0.0).finished();
        if (H2)
            (*H2) = (Matrix(1, 3) << dx / r2, dy / r2, 0.0).finished();
        return (Vector(2) << r2-mr_).finished();
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

int main(){
    return 0;
}