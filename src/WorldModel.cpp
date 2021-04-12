
// #include "WorldModel.h"
#include <gtsam/geometry/Pose3.h>

class Node{
    public:
        Node(gtsam::Pose3 pose);
        ~Node();
    private:
        gtsam::Pose3 _pose;
};

Node::Node(gtsam::Pose3 pose)
{
    _pose = pose;
}

int main(){ 
    // Testing node creation with SE3 pose.
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

    return 0;
}