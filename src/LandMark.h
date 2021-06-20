/**
 * @file   LandMark.h
 * @brief  Defines the LandMark Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#include <tuple>
#include <iostream>
#include <map>
#include <array>

#include "Transform.h"


namespace anloro{

class LandMark
{
public:
    // Constructors
    LandMark(int nodeId, Transform t,
             float sigmaX, float sigmaY, float sigmaZ, float sigmaRoll, float sigmaPitch, float sigmaYaw);
    // The compiler takes care of the default constructor
    LandMark() = default;

    // Member functions
    void AddNode(int nodeId, Transform t,
                 float sigmaX, float sigmaY, float sigmaZ, float sigmaRoll, float sigmaPitch, float sigmaYaw);
    bool ExistsNode(int nodeId);

    typedef std::array<float, 6> Uncertainty; // array with variances x, y, z, roll, pitch, yaw
    typedef std::pair<Transform, Uncertainty> LandMarkData;
    typedef std::pair<int, LandMarkData> RelatedNodesPair; // {node ID, LandMark data}
    typedef std::map<int, LandMarkData> RelatedNodesMap;

    RelatedNodesMap GetRelatedNodes(){return _relatedNodes;};

protected:
    // List of the nodes that percieved the landmark {NodeID, Transform}
    // Note that the Transform must represent the landmark pose relative to the robot frame.
    RelatedNodesMap _relatedNodes;

};

} // namespace anloro