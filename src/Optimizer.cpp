/**
 * @file   Optimizer.cpp
 * @brief  Defines the Optimizer Entity.
 * @author Ángel Lorente Rogel
 * @date   12/09/2021
 */

#include "Optimizer.h"


using namespace gtsam;
using namespace anloro;


anloro::Optimizer::Optimizer(RefFramesMap &refFramesMap, KeyFramesMap &keyFramesMap, 
                            LandMarksMap &landMarksMap, PoseFactorsMap &poseFactorsMap)
{
    _refFramesMap = refFramesMap;
    _keyFramesMap = keyFramesMap;
    _landMarksMap = landMarksMap;
    _poseFactorsMap = poseFactorsMap;
}

anloro::Optimizer::~Optimizer()
{
    //  delete _poseFactorsMap; 
    //  delete _landMarksMap;
    //  delete _keyFramesMap; 
    //  delete _refFramesMap; 
}

void anloro::Optimizer::Optimize()
{
    // --------------------------------------------
    // Build the graph using GTSAM (from our graph)
    // --------------------------------------------

    int constraintCount = 0;

    NonlinearFactorGraph graph;
    Values initialEstimate;
    Values optimizedPoses;

    // Add a prior on the first pose, setting it to the origin
    int initId = 0;
    Pose2 priorPose2 = Pose2(0, 0, 0);
    Pose3 priorPose = Pose3(priorPose2);
    auto priorNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
    graph.addPrior(initId, priorPose, priorNoise);

    Transform transform;
    int idFrom, idTo, NodeId;
    string landMarkId;
    // Add the Pose3 factors
    float sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
    for (std::map<int, PoseFactor *>::const_iterator iter = _poseFactorsMap.begin(); iter != _poseFactorsMap.end(); ++iter)
    {
        idFrom = iter->second->From();
        idTo = iter->second->To();
        transform = iter->second->GetTransform();

        iter->second->GetEulerVariances(sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);

        // auto noiseModel = noiseModel::Diagonal::Variances((Vector(6) << sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw).finished());
        auto noiseModel = noiseModel::Diagonal::Variances((Vector(6) << sigmaRoll, sigmaPitch, sigmaYaw, sigmaX, sigmaY, sigmaZ).finished());
        
        // std::cout << "The variances of the odom are: \n" << sigmaX << ", " << sigmaY << ", " << sigmaZ
        //          << ", " << sigmaRoll << ", " << sigmaPitch << ", " << sigmaYaw << std::endl;

        Pose3 newMean = Pose3(transform.ToMatrix4f().cast<double>()); // Pose3 needs a double datatype

        graph.emplace_shared<BetweenFactor<Pose3>>(idFrom, idTo, newMean, noiseModel);
        constraintCount++;
    }

    // Add the nodes
    for (std::map<int, KeyFrame<int> *>::const_iterator iter = _keyFramesMap.begin(); iter != _keyFramesMap.end(); ++iter)
    {
        NodeId = iter->first;
        transform = iter->second->GetTransform();
        Pose3 newPose = Pose3(transform.ToMatrix4f().cast<double>()); // Pose3 needs a double datatype
        initialEstimate.insert(NodeId, newPose);
    }

    // Add the landmarks
    float x, y, z, dummyVar;
    int nodeId;
    for (std::map<string, LandMark*>::const_iterator iteri = _landMarksMap.begin(); iteri != _landMarksMap.end(); ++iteri)
    {
        landMarkId = iteri->first;
        int landMarkIdInt = landMarkId[1] - '0'; // get rid off the offset for conversion
        Symbol landMarkKey(landMarkId[0], landMarkIdInt);
        // std::cout << "INFO: The generated LM symbol: " << landMarkKey << std::endl;
        
        LandMark::RelatedNodesMap relatedNodes = iteri->second->GetRelatedNodes();
        // Add the constraints to the related nodes
        for (LandMark::RelatedNodesMap::const_iterator iterj = relatedNodes.begin(); iterj != relatedNodes.end(); ++iterj)
        {
            nodeId = iterj->first;
            transform = std::get<0>(iterj->second);
            LandMark::Uncertainty unc = std::get<1>(iterj->second);
            sigmaX = unc[0];
            sigmaY = unc[1];
            sigmaZ = unc[2];
            sigmaRoll = unc[3];
            sigmaPitch = unc[4];
            sigmaYaw = unc[5];

            // auto measurementNoise = noiseModel::Diagonal::Variances((Vector(6) << sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw).finished());
            auto measurementNoise = noiseModel::Diagonal::Variances((Vector(6) << sigmaRoll, sigmaPitch, sigmaYaw, sigmaX, sigmaY, sigmaZ).finished());
            
            // Add transform as a Pose3 BetweenFactor
            Pose3 newMean = Pose3(transform.ToMatrix4f().cast<double>()); // Pose3 needs a double datatype
            graph.emplace_shared<BetweenFactor<Pose3>>(nodeId, landMarkKey, newMean, measurementNoise);
            constraintCount++;
        }

        // As a Pose3
        float xAbs, yAbs, zAbs;
        Pose3 LandMarkPose3 = Pose3(iteri->second->GetInitialEstimate().ToMatrix4f().cast<double>());
        initialEstimate.insert(landMarkKey, LandMarkPose3);
    }

    std::cout << "-------------------------------------------" << std::endl;
    // std::cout << "The initial graph is: " << std::endl;
    // graph.print();
    // initialEstimate.print();

    // --------------------------------------------
    // Configure the optimizer
    // --------------------------------------------

    GaussNewtonParams parameters;
    // LevenbergMarquardtParams parameters;
    int maxIterations = 20; 
    float relativeErrorTol = 1e-5; // Stop iterating if the error is below this value
    parameters.relativeErrorTol = relativeErrorTol;
    parameters.maxIterations = maxIterations;
    GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
    // LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, parameters);
    float iniError = optimizer.error();
    std::cout << "The initial error is: " << iniError << std::endl;

    // --------------------------------------------
    // Optimizer iterating
    // --------------------------------------------

    // Optimize
    int it = 0;
    double lastError = optimizer.error();
    for (int i = 0; i < maxIterations; ++i)
    {
        try
        {
            optimizer.iterate();
            ++it;
        }
        catch (gtsam::IndeterminantLinearSystemException &e)
        {
            std::cout << e.what() << std::endl;
        }
        // early stop condition
        double error = optimizer.error();
        double errorDelta = lastError - error;
        if (i > 0 && errorDelta < relativeErrorTol)
        {
            if (errorDelta < 0)
            {
                // Negative improvement?! Ignore and continue optimizing...
            }
            else
            {
                // Not enough improvement, stop optimization.
                break;
            }
        }
        else if (i == 0 && error < relativeErrorTol)
        {
            // Error is already under relativeErrorTol, stop optimization.
            break;
        }
        lastError = error;
        std::cout << "The error after the iteration " << i << " is: " << error << std::endl;
    }

    // optimizer.values().print("After the optimization: \n *** \n");

    // ----------------------------------------------------------
    // Update the posegraph with the results of the optimization
    // ----------------------------------------------------------

    Symbol graphKey, worldKey;
    // Update the World Model keyframes with the optimized poses.
    for (std::pair<Values::const_iterator, std::map<int, KeyFrame<int> *>::const_iterator> iter(optimizer.values().begin(), _keyFramesMap.begin());
         iter.first != optimizer.values().end() && iter.second != _keyFramesMap.end();
         ++iter.first, ++iter.second)
    {
        // Check if the iterator is in the same node.
        graphKey = (int)iter.first->key;
        worldKey = iter.second->first;
        // std::cout << "GraphKey: " << iter.first->key << " worldKey: " << iter.second->first << std::endl;
        if (graphKey == worldKey)
        {
            // Get the optimized poses from the optimizer
            Pose3 optimizedPose = iter.first->value.cast<Pose3>();
            Eigen::Matrix4f matrix = optimizedPose.matrix().cast<float>();
            Transform transform = Transform(matrix);
            // Update the World Model with the optimized poses
            iter.second->second->SetTransform(transform);
        }
        else
        {
            std::cout << "ERROR: The World model KeyFrame update is failing!" << std::endl;
        }
    }

    // Update the World Model landmark estimate with the optimized poses.
    for (Values::const_iterator iterV = optimizer.values().begin(); iterV != optimizer.values().end(); ++iterV)
    {
        Symbol valueKey = iterV->key;
        for (std::map<string, LandMark *>::const_iterator iterL = _landMarksMap.begin(); iterL != _landMarksMap.end(); ++iterL)
        {
            // Check if the iterator is in the same node.
            landMarkId = iterL->first;
            int landMarkIdInt = landMarkId[1] - '0'; // get rid off the offset for conversion
            Symbol lmKey(landMarkId[0], landMarkIdInt);

            // std::cout << "INFO: Result key: " << valueKey << " LM: " << lmKey << std::endl;
            if (valueKey == lmKey)
            {
                // Pose3
                Pose3 optimizedPose = iterV->value.cast<Pose3>();
                Eigen::Matrix4f matrix = optimizedPose.matrix().cast<float>();
                Transform transform = Transform(matrix);

                // Update the World Model with the optimized poses
                iterL->second->SetInitialEstimate(transform);
                // std::cout << "INFO: The LandMark " << lmKey << " is updated with " << valueKey << " with transform:\n" << transform.ToMatrix4f() << std::endl;
            }
        }
    }

    // ----------------------------------------------------------
    // Get the marginals
    // ----------------------------------------------------------

    try {
        gtsam::Marginals marginals(graph, optimizer.values());
        // gtsam::Matrix info = marginals.marginalCovariance(poses.rbegin()->first);

        // graph.keys().print("This are the keys: \n");
        // marginals.print("Computed marginals: \n");
        float xCov, yCov, zCov, rollCov, pitchCov, yawCov;
        int lastOdomId = _keyFramesMap.rbegin()->first;

        rollCov = marginals.marginalCovariance(lastOdomId)(0);
        pitchCov = marginals.marginalCovariance(lastOdomId)(7);
        yawCov = marginals.marginalCovariance(lastOdomId)(14);
        xCov = marginals.marginalCovariance(lastOdomId)(21);
        yCov = marginals.marginalCovariance(lastOdomId)(28);
        zCov = marginals.marginalCovariance(lastOdomId)(35);

        _lastNodeUnc = Uncertainty{xCov, yCov, zCov, rollCov, pitchCov, yawCov};

        std::cout << "GTSAM: ID: " << _keyFramesMap.rbegin()->first << std::endl;      
        std::cout << "GTSAM (x, y, z, r, p, y): " << xCov << ", " << yCov << ", " << zCov << ", " << rollCov << ", " << pitchCov << ", " << yawCov << std::endl;

        // for (int i=0; i<constraintCount; i++){
        //     // std::cout << "GTSAM: Covariance in variable: " << i+1 << marginals.marginalCovariance(i+1) << std::endl;
        //     xCov = marginals.marginalCovariance(i+1)(0);
        //     yCov = marginals.marginalCovariance(i+1)(7);
        //     zCov = marginals.marginalCovariance(i+1)(14);
        //     rollCov = marginals.marginalCovariance(i+1)(21);
        //     pitchCov = marginals.marginalCovariance(i+1)(28);
        //     yawCov = marginals.marginalCovariance(i+1)(35);

        //     std::cout << "GTSAM: " << xCov << ", " << yCov << ", " << zCov << ", " << rollCov << ", " << pitchCov << ", " << yawCov << std::endl;
        // }


        // if(info.cols() == 6)
        // {
        //     Eigen::Matrix<double, 6, 6> mgtsam = Eigen::Matrix<double, 6, 6>::Identity();
        //     mgtsam.block(3,3,3,3) = info.block(0,0,3,3); // cov rotation
        //     mgtsam.block(0,0,3,3) = info.block(3,3,3,3); // cov translation
        //     mgtsam.block(0,3,3,3) = info.block(0,3,3,3); // off diagonal
        //     mgtsam.block(3,0,3,3) = info.block(3,0,3,3); // off diagonal
        //     memcpy(outputCovariance.data, mgtsam.data(), outputCovariance.total()*sizeof(double));
        // }
        // else
        // {
        //     std::cout << "GTSAM: Could not compute marginal covariance!" << std::endl;
        // }
    }
    catch(gtsam::IndeterminantLinearSystemException & e)
    {
        std::cout << "GTSAM exception caught: \n" << e.what() << std::endl;
    }
    catch(std::exception& e)
    {
        std::cout << "GTSAM exception caught: \n" << e.what() << std::endl;
    }

}
