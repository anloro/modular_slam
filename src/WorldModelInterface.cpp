/**
 * @file   WorldModelInterface.cpp 
 * @brief  World model interface with the front-ends.
 * @author Ángel Lorente Rogel
 * @date   05/05/2021
 */

// my includes
#include "WorldModelInterface.h"
#include <math.h>

using namespace anloro;


anloro::WorldModelInterface::WorldModelInterface(std::string id)
{
    // Get the instance from the World Model
    _worldModel = WorldModel::GetInstance();
    _uniqueID = id;
}

// ---------------------------------------------------------
// --------- Utilities for Front-end interaction -----------
// ---------------------------------------------------------

// Map the input ID into our own internal IDs
int anloro::WorldModelInterface::NodeInternalMapId(int id)
{
    // TO-DO: Add a time check to comapre between ids of different front-ends!
    
    // std::cout << "The input id is: " << id << " with a count of " << _frontEndToModular_node.count(id) << std::endl;


    // Check if the ID is already registered in the WorldModel
    if (_frontEndToModular_node.count(id) == 0)
    {
        // ID not found, increase node count
        // std::cout << "The current node id is: " << _worldModel->currentNodeId << std::endl;

        _worldModel->currentNodeId++;

        // std::cout << "And now is: " << _worldModel->currentNodeId << std::endl;

        // And add it
        _frontEndToModular_node.insert(std::make_pair(id, _worldModel->currentNodeId));
        _modularToFrontEnd_node.insert(std::make_pair(_worldModel->currentNodeId, id));
        return _worldModel->currentNodeId;
    }
    else
    {
        // ID found
        return _frontEndToModular_node[id];
    }
}

// Recover original ID from internal IDs
int anloro::WorldModelInterface::GetNodeIdFromInternalMap(int id)
{
    // Check for the ID
    if (_modularToFrontEnd_node.count(id) == 0)
    {
        // ID not found!
        std::cout << "WARNING: ID " << id << " not found!" << std::endl;
        return -1;
    }
    else
    {
        // ID found
        return _modularToFrontEnd_node[id];
    }
}

// Map the input ID into our own internal IDs
int anloro::WorldModelInterface::LandMarkInternalMapId(int id)
{
    // Check if the ID is already registered in the WorldModel
    if (_frontEndToModular_lm.count(id) == 0)
    {
        // ID not found, increase LandMark count
        _worldModel->currentLandMarkId++;
        // And add it
        _frontEndToModular_lm.insert(std::make_pair(id, _worldModel->currentLandMarkId));
        _modularToFrontEnd_lm.insert(std::make_pair(_worldModel->currentLandMarkId, id));
        return _worldModel->currentLandMarkId;
    }
    else
    {
        // ID found
        return _frontEndToModular_lm[id];
    }
}

// Recover original ID from internal IDs
int anloro::WorldModelInterface::GetLandMarkIdFromInternalMap(int id)
{
    // Check for the ID
    if (_modularToFrontEnd_lm.count(id) == 0)
    {
        // ID not found!
        // std::cout << "WARNING: ID " << id << " not found!" << std::endl;
        return -1;
    }
    else
    {
        // ID found
        return _modularToFrontEnd_lm[id];
    }
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
    internalId = NodeInternalMapId(id);

    KeyFrame<int> *node = new KeyFrame<int>(transform, dummyData);
    _worldModel->AddKeyFrameEntity(internalId, node);
}

// Add a landmark
void anloro::WorldModelInterface::AddLandMark(int landMarkId, Transform transform,
                                              float sigmaX, float sigmaY, float sigmaZ, float sigmaRoll, float sigmaPitch, float sigmaYaw)
{
    int nodeId = _worldModel->currentNodeId;

    // Only consider a landMark if we have odometry
    if(nodeId > 0)
    {
        // Check if the LandMark wasn't previously seen
        if (_frontEndToModular_lm.count(landMarkId) == 0)
        {
            // Get an internal ID for the new landmark
            int internalLandMarkId = LandMarkInternalMapId(landMarkId);

            LandMark *landmark = new LandMark();
            Transform iniStimate;
            // Set the initial estimate for the landmark in absolute coordinates
            std::cout << "Current state: \n"<< _worldModel->currentState.ToMatrix4f() << std::endl;
            std::cout << "LandMark relative pose: \n"<< transform.ToMatrix4f() << std::endl;
            std::cout << "LandMark absolute pose: "<< std::endl;
            iniStimate = _worldModel->currentState * transform;
            std::cout << "Current state * lmRelPose: \n"<< iniStimate.ToMatrix4f() << std::endl;

            landmark->SetInitialEstimate(iniStimate);
            // Add the node from where it was detected
            landmark->AddNode(nodeId, transform,
                              sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);

            _worldModel->AddLandMarkEntity(internalLandMarkId, landmark);
            std::cout << "INFO: Created landmark with id "<< landMarkId << " percieved in node " << nodeId << "!" << std::endl;
            std::cout << "INFO: With relative transform: \n"<< transform.ToMatrix4f() << std::endl;
            std::cout << "INFO: With variances: " 
                      << sigmaX << " " << sigmaY << " " << sigmaZ << " " << sigmaRoll << " " << sigmaPitch << " " << sigmaYaw << std::endl;

        }else{
            // Get the internal ID of the stored landmark
            int internalLandMarkId = LandMarkInternalMapId(landMarkId);
            // And retrive its pointer from the worldmodel
            LandMark *landmark = _worldModel->GetLandMarkEntity(internalLandMarkId);

            // Check if the landmark was seen from an already registered node
            if(landmark->ExistsNode(nodeId)){
                // The robot is probably not moving
                // std::cout << "Node "<< nodeId << " already registered in landMark " << landMarkId << "!" << std::endl;
            }else{
                landmark->AddNode(nodeId, transform,
                                sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);

                // std::cout << "INFO: Added node "<< nodeId << " to landMark " << landMarkId << "!" << std::endl;
                // std::cout << "INFO: With relative transform: \n"<< transform.ToMatrix4f() << std::endl;
                // std::cout << "INFO: With variances: " 
                //           << sigmaX << " " << sigmaY << " " << sigmaZ << " " << sigmaRoll << " " << sigmaPitch << " " << sigmaYaw << std::endl;

                // Optimize after recognizing the same landmark in another node
                if (nodeId - _lastOptimizationNodeId > 1)
                {
                    _worldModel->Optimize();
                    _lastOptimizationNodeId = nodeId;
                }
            }
        }
    }

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
    internalFrom = NodeInternalMapId(fromNode);
    internalTo = NodeInternalMapId(toNode);

    // std::cout << "Pose constraint from id: " << fromNode << " with internal id " << internalFrom << std::endl;
    // std::cout << "To id: " << toNode << " with internal id " << internalTo << std::endl;

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
    
    int nodeId, frontEndId;
    std::map<int, Eigen::Affine3f> mappedOptimizedPoses;
    typedef std::pair<int, Eigen::Affine3f> optimizedPose;

    // Iterate over the KeyFrame's map
    for (std::map<int, Eigen::Affine3f>::const_iterator iter = optimizedPoses.begin(); iter != optimizedPoses.end(); ++iter)
    {
        // Get the information of each node
        nodeId = iter->first;
        frontEndId = GetNodeIdFromInternalMap(nodeId);

        // Check if the ID corresponds to a node from this Front-End
        if (frontEndId > 0)
        {
            mappedOptimizedPoses.insert(optimizedPose(frontEndId, iter->second));
        }
    }

    return mappedOptimizedPoses;
}

// Call the UndoOdometryCorrection function
void anloro::WorldModelInterface::UndoOdometryCorrection(int lastLoopId, Eigen::Matrix4f uncorrection)
{
    int id = NodeInternalMapId(lastLoopId);
    _worldModel->UndoOdometryCorrection(id, uncorrection);
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

// This may be removed after the testing and automatically set with an odometru manager in the world model
void anloro::WorldModelInterface::SetCurrentState(Transform t)
{
    _worldModel->currentState = t;
}