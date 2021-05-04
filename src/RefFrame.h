/**
 * @file   RefFrame.h
 * @brief  Defines the Reference Frame Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#pragma once

#include <tuple>
#include <iostream>

namespace anloro{

class RefFrame
{
public:
    // Constructors
    RefFrame(double x, double y, double z, double roll, double pitch, double yaw);
    // The compiler takes care of the default constructor
    RefFrame() = default;
    // Member functions
    void SetTranslationalVector(double x, double y, double z);
    std::tuple<double, double, double> GetTranslationalVector();
    void SetRotationalVector(double x, double y, double z);
    std::tuple<double, double, double> GetRotationalVector();
    void GetTranslationalAndEulerAngles(double &x, double &y, double &z, double &roll, double &pitch, double &yaw);

protected:
    double _x, _y, _z, _roll, _pitch, _yaw;
};

} // namespace anloro