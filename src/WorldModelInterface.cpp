/**
 * @file   WorldModelInterface.cpp 
 * @brief  World model interface with the front-ends.
 * @author Ángel Lorente Rogel
 * @date   05/05/2021
 */

// my includes
#include "WorldModelInterface.h"

using namespace anloro;


anloro::WorldModelInterface::WorldModelInterface(std::string id)
{
    // Get the instance from the World Model
    _worldModel = WorldModel::GetInstance();
    _uniqueID = id;
}

// ---------------------------------------------------------
// ------------ Front-end interface functions --------------
// ---------------------------------------------------------

// Add a coordinate reference frame to the World model
void anloro::WorldModelInterface::AddRefFrame(float x, float y, float z, float roll, float pitch, float yaw)
{
    Transform transform(x, y, z, roll, pitch, yaw);
    AddRefFrame(transform);
}

void anloro::WorldModelInterface::AddRefFrame(float r11, float r12, float r13, float o14,
                                              float r21, float r22, float r23, float o24,
                                              float r31, float r32, float r33, float o34)
{
    Transform transform(r11, r12, r13, o14,
                        r21, r22, r23, o24,
                        r31, r32, r33, o34);
    AddRefFrame(transform);
}                                           

void anloro::WorldModelInterface::AddRefFrame(Eigen::Matrix4f matrix)
{
    Transform transform(matrix);
    AddRefFrame(transform);
}

void anloro::WorldModelInterface::AddRefFrame(Eigen::Affine3f affineT)
{
    Transform transform(affineT);
    AddRefFrame(transform);
}

void anloro::WorldModelInterface::AddRefFrame(Transform transform)
{
    RefFrame *frame = new RefFrame(transform);
    _worldModel->AddRefFrameEntity(frame);
}

// Add a key-frame to the World model
void anloro::WorldModelInterface::AddKeyFrame(int id, 
                                              float x, float y, float z, 
                                              float roll, float pitch, float yaw)
{
    Transform transform(x, y, z, roll, pitch, yaw);
    AddKeyFrame(id, transform);
}

void anloro::WorldModelInterface::AddKeyFrame(int id, 
                                              float r11, float r12, float r13, float o14,
                                              float r21, float r22, float r23, float o24,
                                              float r31, float r32, float r33, float o34)
{
    Transform transform(r11, r12, r13, o14,
                        r21, r22, r23, o24,
                        r31, r32, r33, o34);
    AddKeyFrame(id, transform);
}

void anloro::WorldModelInterface::AddKeyFrame(int id, Eigen::Matrix4f matrix)
{
    Transform transform(matrix);
    AddKeyFrame(id, transform);
}

void anloro::WorldModelInterface::AddKeyFrame(int id, Eigen::Affine3f affineT)
{
    Transform transform(affineT);
    AddKeyFrame(id, transform);
}

void anloro::WorldModelInterface::AddKeyFrame(int id, Transform transform)
{
    int internalId;
    int dummyData = 0;
    internalId = _worldModel->InternalMapId(id);

    KeyFrame<int> *node = new KeyFrame<int>(transform, dummyData);
    _worldModel->AddKeyFrameEntity(internalId, node);
}

// Add an SE(3) constraint to the World model
void anloro::WorldModelInterface::AddPoseConstraint(int fromNode, int toNode,
                                                    float x, float y, float z,
                                                    float roll, float pitch, float yaw,
                                                    float sigmaX, float sigmaY, float sigmaZ,
                                                    float sigmaRoll, float sigmaPitch, float sigmaYaw)
{
    Transform transform(x, y, z, roll, pitch, yaw);
    AddPoseConstraint(fromNode, toNode,
                      transform,
                      sigmaX, sigmaY, sigmaZ,
                      sigmaRoll, sigmaPitch, sigmaYaw);
}

void anloro::WorldModelInterface::AddPoseConstraint(int fromNode, int toNode, 
                                                    float x, float y, float z, 
                                                    float roll, float pitch, float yaw,
                                                    float sigmaTranslational, float sigmaRotational)
{
    AddPoseConstraint(fromNode, toNode, 
                        x, y, z, roll, pitch, yaw,
                        sigmaTranslational, sigmaTranslational, sigmaTranslational,
                        sigmaRotational, sigmaRotational, sigmaRotational);
}


void anloro::WorldModelInterface::AddPoseConstraint(int fromNode, int toNode, 
                                                    Eigen::Affine3f affineT,
                                                    float sigmaX, float sigmaY, float sigmaZ, 
                                                    float sigmaRoll, float sigmaPitch, float sigmaYaw)
{
    Transform transform(affineT);
    AddPoseConstraint(fromNode, toNode,
                      transform,
                      sigmaX, sigmaY, sigmaZ,
                      sigmaRoll, sigmaPitch, sigmaYaw);
}

void anloro::WorldModelInterface::AddPoseConstraint(int fromNode, int toNode, 
                                                    Eigen::Affine3f affineT, 
                                                    float sigmaTranslational, float sigmaRotational)
{
    AddPoseConstraint(fromNode, toNode,
                      affineT,
                      sigmaTranslational, sigmaTranslational, sigmaTranslational,
                      sigmaRotational, sigmaRotational, sigmaRotational);
}

void anloro::WorldModelInterface::AddPoseConstraint(int fromNode, int toNode, 
                                                    Eigen::Affine3f affineT, Eigen::Matrix<float, 6, 6> noiseModel)
{
    Transform transform(affineT);

    float sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
    sigmaX = noiseModel(0,0);
    sigmaY = noiseModel(1,1);
    sigmaZ = noiseModel(2,2);
    sigmaRoll = noiseModel(3,3);
    sigmaPitch = noiseModel(4,4);
    sigmaYaw = noiseModel(5,5);

    AddPoseConstraint(fromNode, toNode,
                      transform,
                      sigmaX, sigmaY, sigmaZ,
                      sigmaRoll, sigmaPitch, sigmaYaw);
}

void anloro::WorldModelInterface::AddPoseConstraint(int fromNode, int toNode,
                                                    Transform transform,
                                                    float sigmaX, float sigmaY, float sigmaZ,
                                                    float sigmaRoll, float sigmaPitch, float sigmaYaw)
{
    int internalFrom, internalTo;
    internalFrom = _worldModel->InternalMapId(fromNode);
    internalTo = _worldModel->InternalMapId(toNode);

    PoseFactor *poseFactor = new PoseFactor(internalFrom, internalTo,
                                            transform,
                                            sigmaX, sigmaY, sigmaZ,
                                            sigmaRoll, sigmaPitch, sigmaYaw);

    _worldModel->AddPoseFactor(poseFactor);
}

// Get the interface's unique ID
std::string anloro::WorldModelInterface::GetInterfaceId()
{

    return _uniqueID;
}

// Get the optimized poses from the World model
std::map<int, Eigen::Affine3f> anloro::WorldModelInterface::GetOptimizedPoses()
{
    std::map<int, Eigen::Affine3f> optimizedPoses;
    optimizedPoses = _worldModel->GetOptimizedPoses();
    
    return optimizedPoses;
}

// Call the UndoOdometryCorrection function
void anloro::WorldModelInterface::UndoOdometryCorrection(int lastLoopId, Eigen::Matrix4f uncorrection)
{
    _worldModel->UndoOdometryCorrection(lastLoopId, uncorrection);
}

// Call the optimizer for the World model
void anloro::WorldModelInterface::Optimize()
{
    _worldModel->Optimize();
}

// Save the poses into a txt file in Euler format

void anloro::WorldModelInterface::SavePosesRaw()
{
    _worldModel->SavePosesRaw();
}
