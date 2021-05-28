/**
 * @file   main.cpp 
 * @brief  World model interface with the front-ends.
 * @author Ángel Lorente Rogel
 * @date   18/05/2021
 */

// my includes
#include "WorldModelInterface.h"
#include "WorldModelPlotter.h"

#include <thread>

int main()
{
    // --------------------------------------------------------------
    // Graph configuration: -----------------------------------------
    // --------------------------------------------------------------
    // 
    //                      [#]             [#]
    //                   (2, 2, 0)       (4, 2, 0)
    //                    [Node 4]        [Node 3]
    //               
    //    [#]               [#]             [#]      
    // (0, 0, 0)         (2, 0, 0)       (4, 0, 0)    
    // [Node 0]           [Node 1]        [Node 2]    
    // [Global ref.]
    // --------------------------------------------------------------
    // --------------------------------------------------------------

    anloro::WorldModelInterface interface;

    // anloro::WorldModelPlotter plotter;
    // std::thread first(&anloro::WorldModelPlotter::Spin, &plotter);

    // std::vector<float> *xAxis = plotter.GetxAxis();
    // std::vector<float> *yAxis = plotter.GetyAxis();
    // float x[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    // float y[10] = {1, 2, 3, 4, 5, 6, 5, 6, 7, 8};
        
    // for(int i=0; i<10; i++){
    //     xAxis->push_back(x[i]);
    //     yAxis->push_back(y[i]);
    //     std::cin.get();
    // }

    // Add the global reference frame
    interface.AddRefFrame(0, 0, 0, 0, 0, 0);

    // using eigen
    double x, y, z, roll, pitch, yaw;
    Eigen::Matrix3d n;
    n = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix4f m;
    m << n(0,0), n(0,1), n(0,2), x,
         n(1,0), n(1,1), n(1,2), y,
         n(2,0), n(2,1), n(2,2), z,
         0, 0, 0, 1;
    Eigen::Affine3f a = Eigen::Affine3f(m);

    interface.AddPoseConstraint(1, 2, 2, 0, 0, 0, 0, 0, 0.2, 0.2, 0, 0.1, 0.1, 0.1);
    interface.AddPoseConstraint(2, 3, 2, 0, 0, 0, 0, M_PI_2, 0.2, 0.2, 0, 0.1, 0.1, 0.1);
    interface.AddPoseConstraint(3, 4, 2, 0, 0, 0, 0, M_PI_2, 0.2, 0.2, 0, 0.1, 0.1, 0.1);
    interface.AddPoseConstraint(4, 5, 2, 0, 0, 0, 0, M_PI_2, 0.2, 0.2, 0, 0.1, 0.1, 0.1);
    // loop closure
    interface.AddPoseConstraint(5, 2, 2, 0, 0, 0, 0, M_PI_2, 0.2, 0.2, 0, 0.1, 0.1, 0.1);

    // 3. Create the data structure to hold the initialEstimate estimate to the solution
    // For illustrative purposes, these have been deliberately set to incorrect values
    interface.AddKeyFrame(1, 0.5, 0.0, 0, 0, 0, 0.2);
    interface.AddKeyFrame(2, 2.3, 0.1, 0, 0, 0, -0.2);
    interface.AddKeyFrame(3, 4.1, 0.1, 0, 0, 0, M_PI_2);
    interface.AddKeyFrame(4, 4.0, 2.0, 0, 0, 0, M_PI);
    interface.AddKeyFrame(5, 2.1, 2.1, 0, 0, 0, -M_PI_2);

    interface.Optimize();

    interface.SavePosesRaw();

    std::cin.get();
    // first.join();

    return 0;
}