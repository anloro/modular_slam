/**
 * @file   WorldModelInterface.cpp 
 * @brief  World model interface with the front-ends.
 * @author Ángel Lorente Rogel
 * @date   05/05/2021
 */

// my includes
#include "WorldModelInterface.h"

using namespace anloro;

anloro::WorldModelInterface::WorldModelInterface()
{
    // Get the instance from the World Model
    _worldModel = WorldModel::GetInstance();
}

// ---------------------------------------------------------
// ------------ Front-end interface functions --------------
// ---------------------------------------------------------

// Add a coordinate reference frame to the World model
void anloro::WorldModelInterface::AddRefFrame(double x, double y, double z, double roll, double pitch, double yaw)
{
    RefFrame *frame = new RefFrame(x, y, z, roll, pitch, yaw);
    _worldModel->AddRefFrameEntity(frame);
}

// Add a key-frame to the World model
void anloro::WorldModelInterface::AddKeyFrame(int id, double x, double y, double z, double roll, double pitch, double yaw)
{
    int internalId;
    internalId = _worldModel->InternalMapId(id);
    int dummyData = 0;
    KeyFrame<int> *node = new KeyFrame<int>(x, y, z, roll, pitch, yaw, dummyData);
    _worldModel->AddKeyFrameEntity(internalId, node);
}

// Add an SE(3) constraint to the World model using Euler format
void anloro::WorldModelInterface::AddPoseConstraint(int fromNode, int toNode,
                                                    double x, double y, double z,
                                                    double roll, double pitch, double yaw,
                                                    double sigmaX, double sigmaY, double sigmaZ,
                                                    double sigmaRoll, double sigmaPitch, double sigmaYaw)
{
    int internalFrom, internalTo;
    internalFrom = _worldModel->InternalMapId(fromNode);
    internalTo = _worldModel->InternalMapId(toNode);

    PoseFactor *poseFactor = new PoseFactor(internalFrom, internalTo,
                                            x, y, z, roll, pitch, yaw,
                                            sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);
    _worldModel->AddPoseFactor(poseFactor);
}

// Add an SE(3) constraint to the World model using matrix from with Eigen
void anloro::WorldModelInterface::AddPoseConstraint(int fromNode, int toNode, 
                                                    Eigen::Affine3f transform, Eigen::Matrix<double, 6, 6> noiseModel)
{
    int internalFrom, internalTo;
    double x, y, z, roll, pitch, yaw, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
    internalFrom = _worldModel->InternalMapId(fromNode);
    internalTo = _worldModel->InternalMapId(toNode);

    getTranslationAndEulerAngles(transform, x, y, z, roll, pitch, yaw);
    sigmaX = noiseModel(0,0);
    sigmaY = noiseModel(1,1);
    sigmaZ = noiseModel(2,2);
    sigmaRoll = noiseModel(3,3);
    sigmaPitch = noiseModel(4,4);
    sigmaYaw = noiseModel(5,5);
    
    PoseFactor *poseFactor = new PoseFactor(internalFrom, internalTo,
                                            x, y, z, roll, pitch, yaw,
                                            sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);
    _worldModel->AddPoseFactor(poseFactor);
}

void anloro::WorldModelInterface::getTranslationAndEulerAngles(const Eigen::Affine3f &t, double &x, double &y, double &z, double &roll, double &pitch, double &yaw)
{
    x = t(0, 3);
    y = t(1, 3);
    z = t(2, 3);
    roll = atan2f(t(2, 1), t(2, 2));
    pitch = asinf(-t(2, 0));
    yaw = atan2f(t(1, 0), t(0, 0));
}

// Call the optimizer for the World model
void anloro::WorldModelInterface::Optimize()
{
    _worldModel->Optimize();
}

void anloro::WorldModelInterface::SavePosesRaw()
{
    _worldModel->SavePosesRaw();
}
