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

namespace anloro{

class WorldModelInterface
{
    public:
        // CONSTRUCTOR
        WorldModelInterface();

        // MEMBER FUNCTIONS
        void AddRefFrame(double x, double y, double z, double roll, double pitch, double yaw);
        void AddKeyFrame(int id, double x, double y, double z, double roll, double pitch, double yaw);
        void AddPoseConstraint(int fromNode, int toNode, double x, double y, double z, double roll, double pitch, double yaw,
                           double sigmaX, double sigmaY, double sigmaZ, double sigmaRoll, double sigmaPitch, double sigmaYaw);
        void AddPoseConstraint(int fromNode, int toNode, 
                                Eigen::Affine3f transform, Eigen::Matrix<double, 6, 6> noiseModel);
        // from pcl library
        void getTranslationAndEulerAngles(const Eigen::Affine3f &t, double &x, double &y, double &z, double &roll, double &pitch, double &yaw);
        std::map<int, Eigen::Affine3f> GetOptimizedPoses();
        void Optimize();
        void SavePosesRaw(); 
    
    protected: 
        WorldModel *_worldModel;
};


} // namespace anloro
