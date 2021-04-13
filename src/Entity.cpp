#include "Entity.h"

// Get the starting time of the program
auto start_time = std::chrono::system_clock::now();

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

// Entity::~Entity() { delete[] _raw_sensor_data.image; }

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