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

// Add an SE(3) constraint to the World model
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

// Call the optimizer for the World model
void anloro::WorldModelInterface::Optimize()
{
    _worldModel->Optimize();
}

int main()
{
    // --------------------------------------------------------------
    // Graph configuration: -----------------------------------------
    // --------------------------------------------------------------
    //                                       [#]
    //                                    (5, 3, 0)
    //                                    [Node 6]
    //     [#]              [#]                             [#]
    //  (0, 0, 0)        (2, 0, 0)                       (7, 0, 0)
    //  [Node 0]          [Node 1]                        [Node 5]
    //  [Global ref.]
    //                      [#]             [#]             [#]
    //                   (2, -1, 0)      (4, -1, 0)      (7, -2, 0)
    //                    [Node 2]        [Node 3]        [Node 4]
    // --------------------------------------------------------------
    // --------------------------------------------------------------

    WorldModelInterface interface;

    // Add the global reference frame
    interface.AddRefFrame(0, 0, 0, 0, 0, 0);

    // Add the Node 0
    interface.AddKeyFrame(0, 0, 0, 0, 0, 0, 0);

    // Add the Node 1
    interface.AddKeyFrame(1, 2, 0, 0, 0, 0, 0);
    // Add Factor between Node 0 and Node 1
    interface.AddPoseConstraint(0, 1, 2, 0, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);

    // Add the Node 2
    interface.AddKeyFrame(2, 2, -1, 0, 0, 0, 0);
    // Add Factor between Node 1 and Node 2
    interface.AddPoseConstraint(1, 2, 0, -1, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);

    // Add the Node 3
    interface.AddKeyFrame(3, 4, -1, 0, 0, 0, 0);
    // Add Factor between Node 2 and Node 3
    interface.AddPoseConstraint(2, 3, 2, 0, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);

    // Add the Node 4
    interface.AddKeyFrame(4, 7, -2, 0, 0, 0, 0);
    // Add Factor between Node 3 and Node 4
    interface.AddPoseConstraint(3, 4, 3, -1, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);

    // Add the Node 5
    interface.AddKeyFrame(5, 7, 0, 0, 0, 0, 0);
    // Add Factor between Node 4 and Node 5
    interface.AddPoseConstraint(4, 5, 0, 2, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);

    // Add the Node 6
    interface.AddKeyFrame(6, 5, 3, 0, 0, 0, 0);
    // Add Factor between Node 5 and Node 6
    interface.AddPoseConstraint(5, 6, -2, -3, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);

    // Add loop closure constraint
    interface.AddPoseConstraint(6, 1, -3, -3, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);

    interface.Optimize();

    return 0;
}