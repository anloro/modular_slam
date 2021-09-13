/**
 * @file   Optimizer.h
 * @brief  Defines the Optimizer.
 * @author Ángel Lorente Rogel
 * @date   12/09/2021
 */

#pragma once

#include "KeyFrame.h"
#include "LandMark.h"
#include "PoseFactor.h"

#include <Eigen/Geometry>
#include <array>

// includes to the gtsam graph
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/sam/BearingFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>


namespace anloro{

class Optimizer
{
    public:
        typedef std::array<float, 6> Uncertainty; // array with variances x, y, z, roll, pitch, yaw

        // DEFINITIONS FOR EASIER READABILITY
        typedef std::map<int, RefFrame*> RefFramesMap;
        typedef std::map<int, KeyFrame<int>*> KeyFramesMap;
        typedef std::map<string, LandMark*> LandMarksMap;
        typedef std::map<int, PoseFactor*> PoseFactorsMap;

        // Constructors
        Optimizer(RefFramesMap &refFramesMap, KeyFramesMap &keyFramesMap, 
                LandMarksMap &landMarksMap, PoseFactorsMap &poseFactorsMap);
        // Destructor
        ~Optimizer();

        // Member functions
        void Optimize();
        Uncertainty GetLastNodeUncertainty(){return _lastNodeUnc;};

    protected :
        // MAPS DATA 
        RefFramesMap _refFramesMap; // {refframeID, RefFrame*}
        KeyFramesMap _keyFramesMap; // {nodeID, KeyFrame*}
        LandMarksMap _landMarksMap; // {landmarkID, LandMark*}
        PoseFactorsMap _poseFactorsMap; // {posefactorID, PoseFactor*}

        Uncertainty _lastNodeUnc;

};

} // namespace anloro