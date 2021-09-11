/**
 * @file   WorldModel.h
 * @brief  World model declarations and templates.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#pragma once

// my includes
#include "KeyFrame.h"
#include "LandMark.h"
#include "PoseFactor.h"
#include "UdpClientServer.h"

#include "WorldModelPlotter.h"
#include <SFML/Graphics.hpp>
#include <thread>

#include <boost/any.hpp>

// includes to gtsam 
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <iostream>
#include <chrono>
#include <string>


namespace anloro{

class WorldModel
{
    public:
        // CONSTRUCTOR 
        WorldModel();

        // MEMBER FUNCTIONS
        // Front-end utilities
        int NodeInternalMapId(int id);
        int GetNodeIdFromInternalMap(int id);
        int LandMarkInternalMapId(int id);
        int GetLandMarkIdFromInternalMap(int id);
        // Public static function returning a reference to the singleton class:
        static WorldModel *GetInstance();
        // Entity creation
        void AddRefFrameEntity(RefFrame * refFrame);
        void AddKeyFrameEntity(int nodeId, KeyFrame<int> * keyFrame);
        void AddLandMarkEntity(int id, LandMark *LandMark);
        LandMark* GetLandMarkEntity(int id);
        void DeleteKeyFrameEntity(int id);
        void UnregisterKeyFrame(int id);
        void UndoOdometryCorrection(int lastLoopId, int currentLoopId, Eigen::Matrix4f uncorrection);
        void UndoOdometryCorrection(int lastLoopId, Eigen::Matrix4f uncorrection);
        // Factor creation
        void AddPoseFactor(PoseFactor * poseFactor);
        // Optimization
        void Optimize();
        std::map<int, Eigen::Affine3f> GetOptimizedPoses();
        void SavePosesRaw();
        void SavePosesRaw(std::string name);
        void InsertKeyFrameToPlot(KeyFrame<int> *keyFrame);
        void UpdateCompletePlot();

        // DEFINITIONS FOR EASIER READABILITY
        typedef std::map<int, RefFrame*> RefFramesMap;
        typedef std::pair<int, RefFrame*> RefFramePair;
        typedef std::map<int, KeyFrame<int>*> KeyFramesMap;
        typedef std::pair<int, KeyFrame<int>*> KeyFramePair;
        typedef std::map<string, LandMark*> LandMarksMap;
        typedef std::pair<string, LandMark*> LandMarkPair;
        typedef std::map<int, PoseFactor*> PoseFactorsMap;
        typedef std::pair<int, PoseFactor*> PoseFactorPair;

    protected :
        // MAPS DATA 
        RefFramesMap _refFramesMap; // {refframeID, RefFrame*}
        KeyFramesMap _keyFramesMap; // {nodeID, KeyFrame*}
        LandMarksMap _landMarksMap; // {landmarkID, LandMark*}
        PoseFactorsMap _poseFactorsMap; // {posefactorID, PoseFactor*}

        // Plotting thread
        std::thread * _plotterThread;
        WorldModelPlotter _plotter;

        // Communication channel

    
    public:
        // Keeps an internal count of the different entities
        int currentNodeId = -1;
        int currentLandMarkId = -1;
        // static int currentNodeId;
        // static int currentLandMarkId;
        Transform currentState;
        Transform odomCorrection = Transform(0, 0, 0, 0, 0, 0);
        int lastLoopId = -1;
        int _lastOptimizationNodeId = 0;

        // INTERFACE META DATA
        // Map the front-end node id into the modular_slam framework node id
        std::map<int, int> _frontEndToModular_node;
        // Map this framework node id into the front-end node id
        std::map<int, int> _modularToFrontEnd_node;
        // Same for landmarks
        std::map<int, int> _frontEndToModular_lm;
        std::map<int, int> _modularToFrontEnd_lm;
        std::map<int, int> _numberOfTimesSinceFirstDetection;

};


} // namespace anloro
