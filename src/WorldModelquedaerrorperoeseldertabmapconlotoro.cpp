// /**
//  * @file   WorldModel.cpp
//  * @brief  WorldModel definitions.
//  * @author Ángel Lorente Rogel
//  * @date   19/04/2021
//  */

// // my includes
// #include "WorldModel.h"

// #include "SFPlot.h"

// #include <Eigen/Geometry>
// // includes to create the custom graph
// // includes to the gtsam graph
// #include <gtsam/sam/RangeFactor.h>
// #include <gtsam/sam/BearingFactor.h>
// #include <gtsam/slam/BetweenFactor.h>
// #include <gtsam/nonlinear/NonlinearFactorGraph.h>
// #include <gtsam/inference/Key.h>
// #include <gtsam/nonlinear/Values.h>
// #include <gtsam/nonlinear/Marginals.h>
// #include <gtsam/geometry/Pose3.h>
// #include <gtsam/geometry/Rot3.h>
// #include <gtsam/geometry/Pose2.h>

// #include <iostream>
// #include <chrono>
// #include <fstream>

// #include <typeinfo>

// using namespace gtsam;
// using namespace anloro;

// WorldModel* WorldModel::worldModel_= nullptr;

// anloro::WorldModel::WorldModel()
// {
//     _plotterThread = new std::thread(&WorldModelPlotter::Spin, &_plotter);
// }

// WorldModel * WorldModel::GetInstance()
// {
//     if (worldModel_ == nullptr)
//     {
//         worldModel_ = new WorldModel();
//     }
//     return worldModel_;

//     // static WorldModel* instance; // a static local variable is initialized on first call to the
//     //                              // function only, afterwards, it is always the same variable
//     //                              // for every subsequent calls to the function.
//     // return instance;
// };

// // ---------------------------------------------------------
// // --------- Utilities for Front-end interaction -----------
// // ---------------------------------------------------------

// // Map the input ID into our own internal IDs
// int anloro::WorldModel::InternalMapId(int id)
// {
//     // TO-DO: Add a time check to comapre between ids of different front-ends!

//     static int count = 0; // initialized only once across all calls
//     // Keep track of the maps
//     if (_rtabToModular.count(id) == 0)
//     {
//         // ID not found, so add it
//         _rtabToModular.insert(std::make_pair(id, count));
//         _modularToRtab.insert(std::make_pair(count, id));
//         return count++;
//     }
//     else
//     {
//         // ID found
//         return _rtabToModular[id];
//     }
// }

// // Recover original ID from internal IDs
// int anloro::WorldModel::GetIdFromInternalMap(int id)
// {
//     // Check for the ID
//     // if (_modularToRtab.count(id) == 0)
//     // {
//     //     // ID not found!
//     // }
//     // else
//     // {
//     //     // ID found
//     //     return _modularToRtab[id];
//     // }
//     return _modularToRtab[id];
// }

// // ---------------------------------------------------------
// // ------------- Entity creation definitions ---------------
// // ---------------------------------------------------------

// // Add a Reference Frame Entity to the graph
// void anloro::WorldModel::AddRefFrameEntity(RefFrame *refFrame)
// {
//     static int refFrameID = 0; // initialized only once across all calls
//     _refFramesMap.insert(RefFramePair(refFrameID, refFrame));
//     refFrameID++;
// }

// // Add a Key-Frame Entity to the graph
// void anloro::WorldModel::AddKeyFrameEntity(int id, KeyFrame<int> *keyFrame)
// {
//     _keyFramesMap.insert(KeyFramePair(id, keyFrame));
//     // Add to the render window
//     InsertKeyFrameToPlot(keyFrame);
// }

// // Erase Key-Frame Entity from the graph
// void anloro::WorldModel::DeleteKeyFrameEntity(int id)
// {
//     // Check for the ID
//     if (_keyFramesMap.count(id) == 0)
//     {
//         // ID not found!
//     }
//     else
//     {
//         // ID found
//         // Erase the KeyFrame
//         delete _keyFramesMap[id];
//         _keyFramesMap.erase(id);
//         // Update the id keyFrame mapping
//         int otherId = _modularToRtab[id];
//         _modularToRtab.erase(id);
//         _rtabToModular.erase(otherId);
//         // we must also decrease the static count
//         // Delete related poses
//         // Update the render window
//         UpdateCompletePlot();
//     }
// }

// void anloro::WorldModel::UnregisterKeyFrame(int id)
// {
//     for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter != _keyFramesMap.end(); ++iter)
//     {
//         int nodeId = iter->first;
//         if (nodeId > id) 
//         {
//             // Decrease the id of each node after the input id
//             auto nodeHandler = _keyFramesMap.extract(nodeId);
//             nodeHandler.key() = nodeId - 1;
//             _keyFramesMap.insert(std::move(nodeHandler));
//         }
//     }

//     for (std::map<int, int>::const_iterator iter = _modularToRtab.begin(); iter != _modularToRtab.end(); ++iter)
//     {
//         int nodeId = iter->first;
//         if (nodeId > id)
//         {
//             // Decrease the id of each node after the input id
//             auto nodeHandler = _modularToRtab.extract(nodeId);
//             nodeHandler.key() = nodeId - 1;
//             _modularToRtab.insert(std::move(nodeHandler));
//         }
//     }
// }

// // ---------------------------------------------------------
// // ------------- Factor creation definitions ---------------
// // ---------------------------------------------------------

// // Add a pose Factor to the graph
// void anloro::WorldModel::AddPoseFactor(PoseFactor *poseFactor)
// {
//     static int poseFactorID = 0; // initialized only once across all calls
//     _poseFactorsMap.insert(PoseFactorPair(poseFactorID, poseFactor));
//     poseFactorID++;
// }

// // ---------------------------------------------------------
// // ---------------- Interface with Gtsam -------------------
// // ---------------------------------------------------------

// // Optimize the World model (using gtsam)
// void anloro::WorldModel::Optimize()
// {
//     NonlinearFactorGraph graph;
//     Values initialEstimate;
//     Values optimizedPoses;

//     // Add a prior on the first pose, setting it to the origin
//     int initId = 0;
//     Pose2 priorPose2 = Pose2(0, 0, 0);
//     Pose3 priorPose = Pose3(priorPose2);
//     auto priorNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.3, 0.3, 0.0, 0.0, 0.0, 0.1).finished());
//     graph.addPrior(initId, priorPose, priorNoise);

//     // Add the Pose3 factors
//     Eigen::Matrix3d n;
//     int idFrom, idTo, NodeId;
//     double x, y, z, roll, pitch, yaw, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
//     for (std::map<int, PoseFactor *>::const_iterator iter = _poseFactorsMap.begin(); iter != _poseFactorsMap.end(); ++iter)
//     {
//         idFrom = iter->second->From();
//         idTo = iter->second->To();
//         iter->second->GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
//         iter->second->GetEulerVariances(sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);
//         auto noiseModel = noiseModel::Diagonal::Variances((Vector(6) << sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw).finished());
//         n = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
//         Rot3 newR = Rot3(n);
//         // Rot3 newR = Rot3().Yaw(yaw).Pitch(pitch).Roll(roll);
//         Point3 newP = Point3(x, y, z);
//         Pose3 newMean = Pose3(newR, newP);
//         graph.emplace_shared<BetweenFactor<Pose3>>(idFrom, idTo, newMean, noiseModel);
//     }

//     // Add the nodes
//     for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter != _keyFramesMap.end(); ++iter)
//     {
//         NodeId = iter->first;
//         iter->second->GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
//         n = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
//         Rot3 newR = Rot3(n);
//         Point3 newP = Point3(x, y, z);
//         Pose3 newPose = Pose3(newR, newP);
//         initialEstimate.insert(NodeId, newPose);
//     }

//     graph.print("This is the graph:\n");
//     // initialEstimate.print("This is the initial estimate:\n");
//     GaussNewtonParams parameters;
//     // Stop iterating once the change in error between steps is less than this value
//     parameters.relativeErrorTol = 1e-5;
//     // Do not perform more than N iteration steps
//     parameters.maxIterations = 100;
//     GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
//     double err0 = optimizer.error();
//     std::cout << "The error is: " << err0 << std::endl;
//     // optimizedPoses = optimizer.optimize();
//     double err = optimizer.error();
//     std::cout << "The error is: " << err << std::endl;

//     // TRY gtsam approach
//     int it = 0;
//     int iterations = 100;
//     float epsilon = 1e-5;
//     double lastError = optimizer.error();
//     for (int i = 0; i < iterations; ++i)
//     {
//         try
//         {
//             optimizer.iterate();
//             ++it;
//         }
//         catch (gtsam::IndeterminantLinearSystemException &e)
//         {
//             // UWARN("GTSAM exception caught: %s\n Graph has %d edges and %d vertices", e.what(),
//             //   (int)edgeConstraints.size(),
//             //   (int)poses.size());
//             //   delete optimizer;
//             //   return optimizedPoses;
//         }
//         // early stop condition
//         double error = optimizer.error();
//         // UDEBUG("iteration %d error =%f", i + 1, error);
//         double errorDelta = lastError - error;
//         if (i > 0 && errorDelta < epsilon)
//         {
//             if (errorDelta < 0)
//             {
//                 // UDEBUG("Negative improvement?! Ignore and continue optimizing... (%f < %f)", errorDelta, this.epsilon());
//             }
//             else
//             {
//                 // UDEBUG("Stop optimizing, not enough improvement (%f < %f)", errorDelta, this.epsilon());
//                 break;
//             }
//         }
//         else if (i == 0 && error < epsilon)
//         {
//             // UINFO("Stop optimizing, error is already under epsilon (%f < %f)", error, this.epsilon());
//             break;
//         }
//         lastError = error;
//     }
//     optimizedPoses = optimizer.values();

//     // rtabmap approaxh until here

//     // This is for testing
//     // optimizedPoses.print("Optimized poses:\n");
//     // Calculate and print marginal covariances for all variables
//     std::cout.precision(3);
//     Marginals marginals(graph, optimizedPoses);

//     // for (Values::const_iterator iter = optimizer.values().begin(); iter != optimizer.values().end(); ++iter)

//     // Update the World Model with the optimized poses.
//     // for (std::pair<Values::const_iterator, std::map<int, KeyFrame<int> *>::const_iterator> iter(optimizer.values().begin(), _keyFramesMap.begin());
//     //      iter.first != optimizer.values().end() && iter.second != _keyFramesMap.end();
//     //      ++iter.first, ++iter.second)
//     // {
//     //     // First get the optimized poses from the optimizer in Euler format
//     //     int key = (int)iter.first->key;
//     //     Pose3 optimizedPose = iter.first->value.cast<Pose3>();
//     //     x = optimizedPose.x();
//     //     y = optimizedPose.y();
//     //     z = optimizedPose.z();
//     //     Rot3 r = optimizedPose.rotation();
//     //     Vector3 eulerR = r.xyz();
//     //     roll = eulerR[0];
//     //     pitch = eulerR[1];
//     //     yaw = eulerR[2];

//     //     // In case we need the marginals
//     //     // std::cout << "Key-Frame " << key << " has a covariance of:\n"
//     //     //           << marginals.marginalCovariance(key) << std::endl;

//     //     // Then we update the World Model with the optimized poses
//     //     iter.second->second->SetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
//     // }

//     // Now update the links with the marginals and the new constraints
//     // ---------

//     // std::map<int, Eigen::Affine3f> op = GetOptimizedPoses();
//     // for (std::map<int, PoseFactor *>::const_iterator iter = _poseFactorsMap.begin(); iter != _poseFactorsMap.end(); ++iter)
//     // {
//     //     int fromid = iter->second->From();
//     //     int toid = iter->second->To();

//     //     // check if it is an odom constraint
//     //     if (fromid == toid + 1 || fromid == toid - 1)
//     //     {
//     //         Eigen::Affine3f transformTo = op[toid];
//     //         Eigen::Affine3f transformFrom = op[fromid];
//     //         Eigen::Matrix4f tm;
//     //         tm << transformTo(0, 0), transformTo(0, 1), transformTo(0, 2), transformTo(0, 3),
//     //               transformTo(1, 0), transformTo(1, 1), transformTo(1, 2), transformTo(1, 3),
//     //               transformTo(2, 0), transformTo(2, 1), transformTo(2, 2), transformTo(2, 3),
//     //               0, 0, 0, 1;
//     //         Eigen::Matrix4f fm;
//     //         fm << transformFrom(0, 0), transformFrom(0, 1), transformFrom(0, 2), transformFrom(0, 3),
//     //             transformFrom(1, 0), transformFrom(1, 1), transformFrom(1, 2), transformFrom(1, 3),
//     //             transformFrom(2, 0), transformFrom(2, 1), transformFrom(2, 2), transformFrom(2, 3),
//     //             0, 0, 0, 1;
//     //         Eigen::Matrix4f newt = fm.inverse() * tm;

//     //         Eigen::Affine3f t(newt);
//     //         x = t(0, 3);
//     //         y = t(1, 3);
//     //         z = t(2, 3);
//     //         roll = atan2f(t(2, 1), t(2, 2));
//     //         pitch = asinf(-t(2, 0));
//     //         yaw = atan2f(t(1, 0), t(0, 0));

//     //         iter->second->SetTranslationalVector(x,y,z);
//     //         iter->second->SetRotationalVector(roll, pitch, yaw);
//     //     }
//     // }

//     // Let's try to just copy the optimized posegraph
//     NonlinearFactorGraph opgraph;
//     opgraph = optimizer.graph();
//     // opgraph.at(1).get()->linearize().get()->jacobian();
//     // opgraph.at(4).get()->measured();
//     opgraph.print("This is the optimized graph:\n");
//     for (NonlinearFactorGraph::iterator iter = opgraph.begin(); iter != opgraph.end(); ++iter)
//     {
//         int newfromid = iter->get()->front();
//         std::cout << typeid(iter->get()).name() << std::endl;
//         int newtoid = iter->get()->back();
//     }

//     // ---------------------

//     UpdateCompletePlot();
// }

// // Return the optimized poses map
// std::map<int, Eigen::Affine3f> anloro::WorldModel::GetOptimizedPoses()
// {
//     int nodeId, rtabmapId;
//     double x, y, z, roll, pitch, yaw;
//     std::map<int, Eigen::Affine3f> optimizedPoses;
//     typedef std::pair<int, Eigen::Affine3f> optimizedPose;

//     // Iterate over the KeyFrame's map
//     for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter != _keyFramesMap.end(); ++iter)
//     {
//         // Get the information of each node
//         nodeId = iter->first;
//         iter->second->GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
        
//         rtabmapId = GetIdFromInternalMap(nodeId);

//         Eigen::Matrix3d n;
//         n = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
//         Eigen::Matrix4f m;
//         m << n(0, 0), n(0, 1), n(0, 2), x,
//             n(1, 0), n(1, 1), n(1, 2), y,
//             n(2, 0), n(2, 1), n(2, 2), z,
//             0, 0, 0, 1;
//         Eigen::Affine3f transform = Eigen::Affine3f(m);

//         optimizedPoses.insert(optimizedPose(rtabmapId, transform));
//     }

//     return optimizedPoses;
// }

// void anloro::WorldModel::SavePosesRaw()
// {
//     // Create and open a text file
//     std::ofstream outputFile("rawposes.txt");

//     int nodeId;
//     double x, y, z, roll, pitch, yaw;
//     // Iterate over the KeyFrame's map
//     for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter != _keyFramesMap.end(); ++iter)
//     {   
//         // Get the information of each node
//         nodeId = iter->first;
//         iter->second->GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
        
//         // Write to the file
//         outputFile << nodeId << " " << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl;
//     }

//     std::cout << "Raw poses saved as rawposes.txt" << std::endl;

//     // Close the file
//     outputFile.close();
// }

// void anloro::WorldModel::InsertKeyFrameToPlot(KeyFrame<int> *keyFrame)
// {
//     double x, y, z, roll, pitch, yaw;

//     std::vector<float> *xAxis = _plotter.GetxAxis();
//     std::vector<float> *yAxis = _plotter.GetyAxis();
//     keyFrame->GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
//     xAxis->push_back(x);
//     yAxis->push_back(y);
// }

// void anloro::WorldModel::UpdateCompletePlot()
// {
//     double x, y, z, roll, pitch, yaw;

//     std::vector<float> *newxAxis = new std::vector<float>{};
//     std::vector<float> *newyAxis = new std::vector<float>{};
//     // Iterate over the KeyFrame's map
//     for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter != _keyFramesMap.end(); ++iter)
//     {
//         // Get the information of each node
//         iter->second->GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
//         newxAxis->push_back(x);
//         newyAxis->push_back(y);
//     }

//     _plotter.SetAxis(newxAxis, newyAxis);    
// }