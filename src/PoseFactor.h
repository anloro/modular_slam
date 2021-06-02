/**
 * @file   PoseFactor.h
 * @brief  Pose-factor declarations, SE(3) constraints.
 * @author Ángel Lorente Rogel
 * @date   04/05/2021
 */

#pragma once

#include "Transform.h"

namespace anloro{


class PoseFactor
{
public:
    // Constructors
    PoseFactor(int fromNode, int toNode, Transform transform,
               float sigmaX, float sigmaY, float sigmaZ, float sigmaRoll, float sigmaPitch, float sigmaYaw);
    PoseFactor(int fromNode, int toNode, Transform transform, float sigmaTranslational, float sigmaRotational);
    // The compiler takes care of the default constructor
    PoseFactor() = default;

    // Member functions
    void SetTransform(Transform transform);
    Transform GetTransform() {return _transform;};
    void GetEulerVariances(float &sigmaX, float &sigmaY, float &sigmaZ, float &sigmaRoll, float &sigmaPitch, float &sigmaYaw);

    int From();
    int To();

protected:
    int _from, _to;
    Transform _transform;
    float _sigmaX, _sigmaY, _sigmaZ, _sigmaRoll, _sigmaPitch, _sigmaYaw;
};

} // namespace anloro