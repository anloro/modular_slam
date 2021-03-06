/**
 * @file   Transform.cpp
 * @brief  Defines the Transform Entity.
 * @author Ángel Lorente Rogel
 * @date   01/06/2021
 */
#include "Transform.h"


anloro::Transform::Transform(float x, float y, float z, float roll, float pitch, float yaw)
{
    Eigen::Affine3f transform = EulerToAffineTransform(x, y, z, roll, pitch, yaw);

    _affine = transform;
}

anloro::Transform::Transform(float r11, float r12, float r13, float o14,
                             float r21, float r22, float r23, float o24,
                             float r31, float r32, float r33, float o34)
{
    Eigen::Matrix4f m;
    m << r11, r12, r13, o14,
        r21, r22, r23, o24,
        r31, r32, r33, o34,
        0, 0, 0, 1;

    Eigen::Affine3f transform = Eigen::Affine3f(m);

    _affine = transform;
}

anloro::Transform::Transform(Eigen::Matrix4f matrix)
{
    Eigen::Affine3f transform = Eigen::Affine3f(matrix);

    _affine = transform;
}

anloro::Transform::Transform(Eigen::Affine3f transform)
{
    _affine = transform;
}

anloro::Transform::Transform(float x, float y, float z, float qx, float qy, float qz, float qw)
{
	Eigen::Matrix3f rotation = Eigen::Quaternionf(qw, qx, qy, qz).normalized().toRotationMatrix();
    Eigen::Matrix4f m;
    m << rotation(0,0), rotation(0,1), rotation(0,2), x,
         rotation(1,0), rotation(1,1), rotation(1,2), y,
         rotation(2,0), rotation(2,1), rotation(2,2), z,
         0, 0, 0, 1;

    Eigen::Affine3f transform = Eigen::Affine3f(m);

    _affine = transform;
}

// The input (roll, pitch, yaw) must be in radians!
Eigen::Affine3f anloro::Transform::EulerToAffineTransform(float x, float y, float z, float roll, float pitch, float yaw)
{
    Eigen::Matrix3f rot;
    rot = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
	Eigen::Matrix3f rotation = Eigen::Quaternionf(rot).normalized().toRotationMatrix();
    Eigen::Matrix4f m;
    m << rotation(0, 0), rotation(0, 1), rotation(0, 2), x,
         rotation(1, 0), rotation(1, 1), rotation(1, 2), y,
         rotation(2, 0), rotation(2, 1), rotation(2, 2), z,
         0, 0, 0, 1;

    Eigen::Affine3f transform = Eigen::Affine3f(m);

    return transform;
}

Eigen::Matrix4f anloro::Transform::ToMatrix4f()
{
    Eigen::Matrix4f matrix;
    matrix << _affine(0, 0), _affine(0, 1), _affine(0, 2), _affine(0, 3),
              _affine(1, 0), _affine(1, 1), _affine(1, 2), _affine(1, 3),
              _affine(2, 0), _affine(2, 1), _affine(2, 2), _affine(2, 3),
              0, 0, 0, 1;

    return matrix;
}

void anloro::Transform::SetTranslationalAndEulerAngles(float x, float y, float z, float roll, float pitch, float yaw)
{
    Eigen::Affine3f transform = EulerToAffineTransform(x, y, z, roll, pitch, yaw);

    _affine = transform;
}

void anloro::Transform::SetTranslationalVector(float x, float y, float z)
{
    _affine(0, 3) = x;
    _affine(1, 3) = y;
    _affine(2, 3) = z;
}

void anloro::Transform::SetEulerAngles(float roll, float pitch, float yaw)
{
    Eigen::Matrix3f rot;
    rot = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) 
            * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) 
            * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
	Eigen::Matrix3f rotation = Eigen::Quaternionf(rot).normalized().toRotationMatrix();

    _affine(0, 0) = rotation(0, 0);
    _affine(0, 1) = rotation(0, 1);
    _affine(0, 2) = rotation(0, 2);
    _affine(1, 0) = rotation(1, 0);
    _affine(1, 1) = rotation(1, 1);
    _affine(1, 2) = rotation(1, 2);
    _affine(2, 0) = rotation(2, 0);
    _affine(2, 1) = rotation(2, 1);
    _affine(2, 2) = rotation(2, 2);
}

void anloro::Transform::GetTranslationalAndEulerAngles(float &x, float &y, float &z, float &roll, float &pitch, float &yaw)
{
    x = _affine(0, 3);
    y = _affine(1, 3);
    z = _affine(2, 3);
    roll = atan2f(_affine(2, 1), _affine(2, 2));
    pitch = asinf(-_affine(2, 0));
    yaw = atan2f(_affine(1, 0), _affine(0, 0));
}

void anloro::Transform::GetTranslationalVector(float &x, float &y, float &z)
{
    x = _affine(0, 3);
    y = _affine(1, 3);
    z = _affine(2, 3);
}

void anloro::Transform::GetEulerAngles(float &roll, float &pitch, float &yaw)
{
    roll = atan2f(_affine(2, 1), _affine(2, 2));
    pitch = asinf(-_affine(2, 0));
    yaw = atan2f(_affine(1, 0), _affine(0, 0));
}

float anloro::Transform::X()
{
    return _affine(0, 3);
}

float anloro::Transform::Y()
{
    return _affine(1, 3);
}

float anloro::Transform::Z()
{
    return _affine(2, 3);
}

anloro::Transform anloro::Transform::inverse()
{
	bool invertible = false;
	Eigen::Matrix4f inverse;
	Eigen::Matrix4f::RealScalar det;
	Transform(_affine).ToMatrix4f().computeInverseAndDetWithCheck(inverse, det, invertible);
	if(!invertible){
        std::cout << "WARNING! The Transform is not invertible!" << std::endl;
    }

	return Transform(inverse);
}

anloro::Transform anloro::Transform::operator*(Transform t) const
{
	Eigen::Affine3f m = Eigen::Affine3f(Transform(_affine).ToMatrix4f()*t.ToMatrix4f());
	// make sure rotation is always normalized!
	m.linear() = Eigen::Quaternionf(m.linear()).normalized().toRotationMatrix();
	return Transform(m);
}

anloro::Transform anloro::Transform::Clone()
{
	return Transform(_affine);
}