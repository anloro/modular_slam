/**
 * @file   PoseFactor.cpp
 * @brief  Pose-factor declarations, SE(3) constraints.
 * @author Ángel Lorente Rogel
 * @date   04/05/2021
 */
#include "PoseFactor.h"

anloro::PoseFactor::PoseFactor(int fromNode, int toNode, double x, double y, double z, double roll, double pitch, double yaw,
                             double sigmaX, double sigmaY, double sigmaZ, double sigmaRoll, double sigmaPitch, double sigmaYaw)
{
    PoseFactor::_from = fromNode;
    PoseFactor::_to = toNode;
    PoseFactor::_x = x;
    PoseFactor::_y = y;
    PoseFactor::_z = z;
    PoseFactor::_roll = roll;
    PoseFactor::_pitch = pitch;
    PoseFactor::_yaw = yaw;
    PoseFactor::_sigmaX = sigmaX;
    PoseFactor::_sigmaY = sigmaY;
    PoseFactor::_sigmaZ = sigmaZ;
    PoseFactor::_sigmaRoll = sigmaRoll;
    PoseFactor::_sigmaPitch = sigmaPitch;
    PoseFactor::_sigmaYaw = sigmaYaw;
}

void anloro::PoseFactor::SetTranslationalVector(double x, double y, double z)
{
    PoseFactor::_x = x;
    PoseFactor::_y = y;
    PoseFactor::_z = z;
}

void anloro::PoseFactor::SetRotationalVector(double roll, double pitch, double yaw)
{
    PoseFactor::_roll = roll;
    PoseFactor::_pitch = pitch;
    PoseFactor::_yaw = yaw;
}

void anloro::PoseFactor::GetTranslationalAndEulerAngles(double &x, double &y, double &z, double &roll, double &pitch, double &yaw)
{
    x = PoseFactor::_x;
    y = PoseFactor::_y;
    z = PoseFactor::_z;
    roll = PoseFactor::_roll;
    pitch = PoseFactor::_pitch;
    yaw = PoseFactor::_yaw;
}

void anloro::PoseFactor::GetEulerVariances(double &sigmaX, double &sigmaY, double &sigmaZ, double &sigmaRoll, double &sigmaPitch, double &sigmaYaw)
{
    sigmaX = PoseFactor::_sigmaX;
    sigmaY = PoseFactor::_sigmaY;
    sigmaZ = PoseFactor::_sigmaZ;
    sigmaRoll = PoseFactor::_sigmaRoll;
    sigmaPitch = PoseFactor::_sigmaPitch;
    sigmaYaw = PoseFactor::_sigmaYaw;
}

int anloro::PoseFactor::From()
{
    return _from;
}

int anloro::PoseFactor::To()
{
    return _to;
}