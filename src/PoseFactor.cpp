/**
 * @file   PoseFactor.cpp
 * @brief  Pose-factor declarations, SE(3) constraints.
 * @author Ángel Lorente Rogel
 * @date   04/05/2021
 */


#include "PoseFactor.h"

anloro::PoseFactor::PoseFactor(int fromNode, int toNode, Transform transform,
                               float sigmaX, float sigmaY, float sigmaZ, float sigmaRoll, float sigmaPitch, float sigmaYaw)
{
    _from = fromNode;
    _to = toNode;
    _transform = transform;
    _sigmaX = sigmaX;
    _sigmaY = sigmaY;
    _sigmaZ = sigmaZ;
    _sigmaRoll = sigmaRoll;
    _sigmaPitch = sigmaPitch;
    _sigmaYaw = sigmaYaw;
}

anloro::PoseFactor::PoseFactor(int fromNode, int toNode, Transform transform, float sigmaTranslational, float sigmaRotational)
{
    *this = PoseFactor(fromNode, toNode, transform,
                       sigmaTranslational, sigmaTranslational, sigmaTranslational,
                       sigmaRotational, sigmaRotational, sigmaRotational);
}

void anloro::PoseFactor::GetEulerVariances(float &sigmaX, float &sigmaY, float &sigmaZ, float &sigmaRoll, float &sigmaPitch, float &sigmaYaw)
{
    sigmaX = _sigmaX;
    sigmaY = _sigmaY;
    sigmaZ = _sigmaZ;
    sigmaRoll = _sigmaRoll;
    sigmaPitch = _sigmaPitch;
    sigmaYaw = _sigmaYaw;
}

void anloro::PoseFactor::SetTransform(Transform transform)
{
    _transform = transform;
}

int anloro::PoseFactor::From()
{
    return _from;
}

int anloro::PoseFactor::To()
{
    return _to;
}