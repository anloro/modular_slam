/**
 * @file   RefFrame.h
 * @brief  Defines the Reference Frame Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#pragma once

#include "LandMark.h"

class RefFrame : public LandMark
{
public:
    // Constructors
    RefFrame(double x, double y, double z, double roll, double pitch, double yaw);
    // The compiler takes care of the default constructor
    RefFrame() = default;
    // Member functions
    void SetRotationalVector(double x, double y, double z);
    std::tuple<double, double, double> GetRotationalVector();

protected:
    double _roll, _pitch, _yaw;
};