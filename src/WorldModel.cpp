
// #include "WorldModel.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
// #include <gtsam/geometry/Pose2.h>
// #include <gtsam/inference/Key.h>
// #include <gtsam/nonlinear/GaussNewtonOptimizer.h>
// #include <gtsam/nonlinear/Marginals.h>

#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>

// Get the starting time of the program
auto start_time = std::chrono::system_clock::now();

class Entity{
    public:
        Entity();
        Entity(int id, gtsam::Pose3 pose);
        Entity(int id, gtsam::Pose3 pose, int timestamp, cv::Mat image, cv::Mat depth);
        // ~Entity();

        struct raw_sensor_data{
            // unique timestamp
            int timestamp;
            // visual data
            cv::Mat image;
            cv::Mat depth;  // depth information from the 2D image
            // lidar data
            
        };

    private:
        gtsam::Pose3 _pose;
        int _id;
        raw_sensor_data _raw_sensor_data;
};

// Entity::~Entity() { delete[] _raw_sensor_data.image; }

Entity::Entity()
{
    // Get the ID by computing the timestamp.
    std::chrono::duration<double> diff; // maybe I have to change this to be int
    int id; 
    auto current_time = std::chrono::system_clock::now();

    diff = current_time - start_time;
    id = diff.count();

    _id = id;
    // _pose = Null;
}

Entity::Entity(int id, gtsam::Pose3 pose)
{
    _id = id;
    _pose = pose;
}

Entity::Entity(int id, gtsam::Pose3 pose, int timestamp, cv::Mat image, cv::Mat depth)
{
    _id = id;
    _pose = pose;
    _raw_sensor_data.timestamp = timestamp;
    _raw_sensor_data.image = image;
    _raw_sensor_data.depth = depth;
}

template <class T, class P>
class Factor{
    // It specifies the prior between nodes
    public:
        Factor(T mean, P noise);

    private:
        // specify between which nodes 
        int _from_node;
        int _to_node;
        // specify the mean
        T _mean;
        // specify noise model/uncertainty of the factor
        P _noise;
};

template <class T, class P>
Factor<T, P>::Factor(T mean, P noise)
{
    _mean = mean;
    _noise = noise;
}

class WorldModel{
    public:
        WorldModel();
        void AddEntity();
        void AddFactor();

    private:
        // save the info of the custom pose graph
        // Entity _entities;
        // Factor _factors;
        // save the info for gtsam
        gtsam::NonlinearFactorGraph _graph;
        gtsam::Values _initial_estimate;
};

WorldModel::WorldModel(){
    // do we create the graph here?
}

void WorldModel::AddEntity()
{
    // add new entity
}

void WorldModel::AddFactor()
{
    // add new factor
}

int main()
{

    // Testing entity creation with SE3 pose.
    gtsam::Point3 t;
    t = gtsam::Point3(0.0, 0.0, 0.0);
    std::cout<< t << std::endl;
    
    gtsam::Rot3 r;
    r = gtsam::Rot3(0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);
    std::cout << r << std::endl;

    gtsam::Pose3 new_pose;
    new_pose = gtsam::Pose3(r, t);
    std::cout << new_pose << std::endl;

    // Test the Entity constructors
    Entity node0; 
    node0 = Entity();


    return 0;
}