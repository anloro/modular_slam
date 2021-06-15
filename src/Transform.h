/**
 * @file   Transform.h
 * @brief  Defines the Transform.
 * @author Ángel Lorente Rogel
 * @date   01/06/2021
 */

#pragma once

#include <Eigen/Geometry>

namespace anloro{

class Transform
{
public:
    // Constructors
    Transform(float x, float y, float z, float roll, float pitch, float yaw);
    // rotation matrix r__ and origin o__
    Transform(float r11, float r12, float r13, float o14,
              float r21, float r22, float r23, float o24,
              float r31, float r32, float r33, float o34);
    Transform(Eigen::Matrix4f matrix);
    Transform(Eigen::Affine3f transform);
    // Using quaternions
    Transform(float x, float y, float z, float qx, float qy, float qz, float qw);
    // The compiler takes care of the default constructor
    Transform() = default;

    // Member functions
    Eigen::Affine3f EulerToAffineTransform(float x, float y, float z, float roll, float pitch, float yaw);
    Eigen::Matrix4f ToMatrix4f();

    void SetTranslationalAndEulerAngles(float x, float y, float z, float roll, float pitch, float yaw);
    void GetTranslationalAndEulerAngles(float &x, float &y, float &z, float &roll, float &pitch, float &yaw);
    Eigen::Affine3f GetAffineTransform() {return _affine;};

protected:
    // Transform _transform;
    Eigen::Affine3f _affine;
};

} // namespace anloro