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
    private:
        // CONSTRUCTOR (private to make it a singleton)
        WorldModel();

    protected:
        static WorldModel* worldModel_;

    public:
        // Singletons should not be cloneable.
        WorldModel(WorldModel const&) = delete;
        // Singletons should not be assignable.
        void operator = (WorldModel const&) = delete;

        // MEMBER FUNCTIONS
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
    
    public:
        // Keeps an internal count of the different entities
        int currentNodeId = -1;
        int currentLandMarkId = -1;
        // static int currentNodeId;
        // static int currentLandMarkId;
        Transform currentState;
        Transform odomCorrection = Transform(0, 0, 0, 0, 0, 0);
        int lastLoopId = -1;
};


} // namespace anloro
