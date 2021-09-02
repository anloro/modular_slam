/**
 * @file   WorldModelInterface.h
 * @brief  World model interface with the front-ends.
 * @author Ángel Lorente Rogel
 * @date   05/05/2021
 */

#pragma once

// my includes
#include "WorldModel.h"
#include <Eigen/Geometry>
#include <string>

namespace anloro{

class WorldModelInterface
{
    public:
        // CONSTRUCTOR
        WorldModelInterface() : WorldModelInterface("GenericId") {};
        WorldModelInterface(std::string id);

        // MEMBER FUNCTIONS
        // Front-end utilities
        int NodeInternalMapId(int id);
        int GetNodeIdFromInternalMap(int id);
        int LandMarkInternalMapId(int id);
        int GetLandMarkIdFromInternalMap(int id);
        // Add a reference frame
        void AddRefFrame(float x, float y, float z, float roll, float pitch, float yaw);

        void AddRefFrame(float r11, float r12, float r13, float o14,
                         float r21, float r22, float r23, float o24,
                         float r31, float r32, float r33, float o34);

        void AddRefFrame(Eigen::Matrix4f matrix);

        void AddRefFrame(Eigen::Affine3f affineT);

        void AddRefFrame(Transform transform);

        // Add a Key-frame
        void AddKeyFrame(double timeStamp,int id, float x, float y, float z, float roll, float pitch, float yaw);

        void AddKeyFrame(double timeStamp,int id, 
                         float r11, float r12, float r13, float o14,
                         float r21, float r22, float r23, float o24,
                         float r31, float r32, float r33, float o34);

        void AddKeyFrame(double timeStamp,int id, Eigen::Matrix4f matrix);

        void AddKeyFrame(double timeStamp,int id, Eigen::Affine3f affineT);

        void AddKeyFrame(double timeStamp,int id, Transform transform);

        // Add a landmark
        void AddLandMark(int landMarkId, Transform transform,
                         float sigmaX, float sigmaY, float sigmaZ, float sigmaRoll, float sigmaPitch, float sigmaYaw);

        // Add a pose constraint
        void AddPoseConstraint(int fromNode, int toNode, 
                                float x, float y, float z, 
                                float roll, float pitch, float yaw,
                                float sigmaX, float sigmaY, float sigmaZ, 
                                float sigmaRoll, float sigmaPitch, float sigmaYaw);

        void AddPoseConstraint(int fromNode, int toNode, 
                                float x, float y, float z, 
                                float roll, float pitch, float yaw,
                                float sigmaTranslational, 
                                float sigmaRotational);

        void AddPoseConstraint(int fromNode, int toNode, 
                                Eigen::Affine3f affineT,
                                float sigmaX, float sigmaY, float sigmaZ, 
                                float sigmaRoll, float sigmaPitch, float sigmaYaw);

        void AddPoseConstraint(int fromNode, int toNode, 
                                Eigen::Affine3f affineT, 
                                float sigmaTranslational, 
                                float sigmaRotational);

        void AddPoseConstraint(int fromNode, int toNode,
                                Eigen::Affine3f affineT,
                                Eigen::Matrix<float, 6, 6> noiseModel);

        void AddPoseConstraint(int fromNode, int toNode,
                               Transform transform,
                               float sigmaX, float sigmaY, float sigmaZ,
                               float sigmaRoll, float sigmaPitch, float sigmaYaw);

        // Get the interface's ID
        std::string GetInterfaceId();

        // Get the optimized poses from the World model
        std::map<int, Eigen::Affine3f> GetOptimizedPoses();

        // Undo odometry correction call
        void UndoOdometryCorrection(int lastLoopId, Eigen::Matrix4f uncorrection);

        // Optimize the pose-graph
        void Optimize();

        // Save the poses into a txt file in Euler format
        void SavePosesRaw(); 
        void SavePosesRaw(std::string name); 

        // May be removed later
        void SetCurrentState(Transform t);
    
    protected: 
        WorldModel *_worldModel;
        std::string _uniqueID;

        // INTERFACE META DATA
        // Map the front-end node id into the modular_slam framework node id
        std::map<int, int> _frontEndToModular_node;
        // Map this framework node id into the front-end node id
        std::map<int, int> _modularToFrontEnd_node;
        // Same for landmarks
        std::map<int, int> _frontEndToModular_lm;
        std::map<int, int> _modularToFrontEnd_lm;
        std::map<int, int> _numberOfTimesSinceFirstDetection;
        int _lastOptimizationNodeId = 0;

};


} // namespace anloro
