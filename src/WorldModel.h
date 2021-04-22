// my includes
// #include "Entity.h"

// includes to create the custom graph
#include <opencv2/core/core.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
// includes to the gtsam graph
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
// #include <gtsam/geometry/Pose2.h>
// #include <gtsam/inference/Key.h>
// #include <gtsam/nonlinear/Marginals.h>

#include <iostream>
#include <chrono>

class WorldModel
{
    public:
        // Constructor
        WorldModel();

        template <typename ENTITYTYPE, class T, class U, class V>
        class EntityContainer{

            public:
                ENTITYTYPE myentity;
                T mydatat;
                U mydatau;
                V mydatav;
                // void AddEntity(int nodeId, double x, double y, double z);
        };

        // template <typename ENTITYTYPE>
        // class EntityContainer <LandMark>
        // {
        //     void AddEntity(int nodeId, double x, double y, double z);
        // };

        // Member functions
        template <typename EntityContainer>
        void AddEntity(int nodeId, double x, double y, double z, double roll, double pitch, double yaw, T data);

        // 2D case
        // void AddEntity(int nodeId, double x, double y, double theta);
        // 3D case
        template <typename ENTITYTYPE>
        void AddEntity(int nodeId, double x, double y, double z, double roll, double pitch, double yaw);
        template <class ENTITYTYPE, class T> 
        void AddEntity(int nodeId, double x, double y, double z, double roll, double pitch, double yaw, T data);
        
        void AddFactor(int fromNode, int toNode, double x, double y, double theta, double sigmaX, double sigmaY, double sigmaTheta);
        void AddFactor(int fromNode, int toNode, double x, double y, double z, double roll, double pitch, double yaw, double sigmaX, double sigmaY, double sigmaZ, double sigmaRoll, double sigmaPitch, double sigmaYaw);
        // Optimization
        void Optimize();

    // private :
        // save the info of the custom pose graph
        // std::vector<Entity> _entities;
        // Factor _factors;

        // save the info for gtsam
        gtsam::NonlinearFactorGraph _graph;
        gtsam::Values _initialEstimate;
        gtsam::Values _result;
};