/**
 * @file   LandMark.cpp
 * @brief  Defines the LandMark Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#include "LandMark.h"


anloro::LandMark::LandMark(Transform t)
{
    _initialStimate = t;
}

void anloro::LandMark::AddNode(int nodeId, Transform t,
                               float sigmaX, float sigmaY, float sigmaZ, float sigmaRoll, float sigmaPitch, float sigmaYaw)
{
    Uncertainty unc{sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw};
    LandMarkData lm(t, unc);
    _relatedNodes.insert(RelatedNodesPair(nodeId, lm));
}

void anloro::LandMark::SetInitialEstimate(Transform t)
{
    _initialStimate = t;
}

bool anloro::LandMark::ExistsNode(int nodeId)
{
    // Check if the nodeID is already registered in this landmark
    if (_relatedNodes.count(nodeId) == 0){
        // ID not found
        return false;
    }
    else{
        // ID found
        return true;
    }
}