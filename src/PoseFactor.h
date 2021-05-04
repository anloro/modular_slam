/**
 * @file   PoseFactor.h
 * @brief  Pose-factor declarations, SE(3) constraints.
 * @author Ángel Lorente Rogel
 * @date   04/05/2021
 */

#pragma once

namespace anloro{


class PoseFactor
{
public:
    // Constructors
    PoseFactor(int fromNode, int toNode, double x, double y, double z, double roll, double pitch, double yaw, 
                double sigmaX, double sigmaY, double sigmaZ, double sigmaRoll, double sigmaPitch, double sigmaYaw);
    // The compiler takes care of the default constructor
    PoseFactor() = default;
    // Member functions
    void SetTranslationalVector(double x, double y, double z);
    void SetRotationalVector(double x, double y, double z);
    void GetTranslationalAndEulerAngles(double &x, double &y, double &z, double &roll, double &pitch, double &yaw);

protected:
    int _from, _to;
    double _x, _y, _z, _roll, _pitch, _yaw, _sigmaX, _sigmaY, _sigmaZ, _sigmaRoll, _sigmaPitch, _sigmaYaw;
};

} // namespace anloro