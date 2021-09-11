/**
 * @file   WorldModelInterface.cpp 
 * @brief  World model interface with the front-ends.
 * @author Ángel Lorente Rogel
 * @date   05/05/2021
 */

// my includes
#include "WorldModelInterface.h"
#include <math.h>

using namespace anloro;


anloro::WorldModelInterface::WorldModelInterface(std::string id)
{
    // Get the instance from the World Model
    // _worldModel = WorldModel::GetInstance();
    client = UdpClient();
    _uniqueID = id;
}

// ---------------------------------------------------------
// --------- Utilities for Front-end interaction -----------
// ---------------------------------------------------------

// ---------------------------------------------------------
// ------------ Front-end interface functions --------------
// ---------------------------------------------------------

// Add a coordinate reference frame to the World model
void anloro::WorldModelInterface::AddRefFrame(float x, float y, float z, float roll, float pitch, float yaw)
{
    Transform transform(x, y, z, roll, pitch, yaw);
    AddRefFrame(transform);
}

void anloro::WorldModelInterface::AddRefFrame(float r11, float r12, float r13, float o14,
                                              float r21, float r22, float r23, float o24,
                                              float r31, float r32, float r33, float o34)
{
    Transform transform(r11, r12, r13, o14,
                        r21, r22, r23, o24,
                        r31, r32, r33, o34);
    AddRefFrame(transform);
}                                           

void anloro::WorldModelInterface::AddRefFrame(Eigen::Matrix4f matrix)
{
    Transform transform(matrix);
    AddRefFrame(transform);
}

void anloro::WorldModelInterface::AddRefFrame(Eigen::Affine3f affineT)
{
    Transform transform(affineT);
    AddRefFrame(transform);
}

void anloro::WorldModelInterface::AddRefFrame(Transform transform)
{
    MsgUdp newMsg;
    newMsg.type = 'a';
    float x, y, z, pitch, yaw, roll; 
    transform.GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
    newMsg.element.refFrame.pose.x = x;
    newMsg.element.refFrame.pose.y = y;
    newMsg.element.refFrame.pose.z = z;
    newMsg.element.refFrame.pose.roll = roll;
    newMsg.element.refFrame.pose.pitch = pitch;
    newMsg.element.refFrame.pose.yaw = yaw;
    client.Send(newMsg);

}

// Add a key-frame to the World model
void anloro::WorldModelInterface::AddKeyFrame(double timeStamp, int id, 
                                              float x, float y, float z, 
                                              float roll, float pitch, float yaw)
{
    Transform transform(x, y, z, roll, pitch, yaw);
    AddKeyFrame(timeStamp, id, transform);
}

void anloro::WorldModelInterface::AddKeyFrame(double timeStamp, int id, 
                                              float r11, float r12, float r13, float o14,
                                              float r21, float r22, float r23, float o24,
                                              float r31, float r32, float r33, float o34)
{
    Transform transform(r11, r12, r13, o14,
                        r21, r22, r23, o24,
                        r31, r32, r33, o34);
    AddKeyFrame(timeStamp, id, transform);
}

void anloro::WorldModelInterface::AddKeyFrame(double timeStamp, int id, Eigen::Matrix4f matrix)
{
    Transform transform(matrix);
    AddKeyFrame(timeStamp, id, transform);
}

void anloro::WorldModelInterface::AddKeyFrame(double timeStamp, int id, Eigen::Affine3f affineT)
{
    Transform transform(affineT);
    AddKeyFrame(timeStamp, id, transform);
}

void anloro::WorldModelInterface::AddKeyFrame(double timeStamp, int id, Transform transform)
{
    MsgUdp newMsg;
    newMsg.type = 'b';
    float x, y, z, pitch, yaw, roll; 
    transform.GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
    newMsg.element.keyFrame.pose.x = x;
    newMsg.element.keyFrame.pose.y = y;
    newMsg.element.keyFrame.pose.z = z;
    newMsg.element.keyFrame.pose.roll = roll;
    newMsg.element.keyFrame.pose.pitch = pitch;
    newMsg.element.keyFrame.pose.yaw = yaw;
    newMsg.element.keyFrame.id = id;
    newMsg.element.keyFrame.timeStamp = timeStamp;
    client.Send(newMsg);
}

// Add a landmark
void anloro::WorldModelInterface::AddLandMark(int landMarkId, Transform transform,
                                              float sigmaX, float sigmaY, float sigmaZ, float sigmaRoll, float sigmaPitch, float sigmaYaw)
{
    MsgUdp newMsg;
    newMsg.type = 'c';
    float x, y, z, pitch, yaw, roll; 
    transform.GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
    newMsg.element.landmark.pose.x = x;
    newMsg.element.landmark.pose.y = y;
    newMsg.element.landmark.pose.z = z;
    newMsg.element.landmark.pose.roll = roll;
    newMsg.element.landmark.pose.pitch = pitch;
    newMsg.element.landmark.pose.yaw = yaw;
    newMsg.element.landmark.landMarkId = landMarkId;
    newMsg.element.landmark.unc.sigmaX = sigmaX;
    newMsg.element.landmark.unc.sigmaY = sigmaY;
    newMsg.element.landmark.unc.sigmaZ = sigmaZ;
    newMsg.element.landmark.unc.sigmaRoll = sigmaRoll;
    newMsg.element.landmark.unc.sigmaPitch = sigmaPitch;
    newMsg.element.landmark.unc.sigmaYaw = sigmaYaw;
    client.Send(newMsg);

}

// Add an SE(3) constraint to the World model
void anloro::WorldModelInterface::AddPoseConstraint(int fromNode, int toNode,
                                                    float x, float y, float z,
                                                    float roll, float pitch, float yaw,
                                                    float sigmaX, float sigmaY, float sigmaZ,
                                                    float sigmaRoll, float sigmaPitch, float sigmaYaw)
{
    Transform transform(x, y, z, roll, pitch, yaw);
    AddPoseConstraint(fromNode, toNode,
                      transform,
                      sigmaX, sigmaY, sigmaZ,
                      sigmaRoll, sigmaPitch, sigmaYaw);
}

void anloro::WorldModelInterface::AddPoseConstraint(int fromNode, int toNode, 
                                                    float x, float y, float z, 
                                                    float roll, float pitch, float yaw,
                                                    float sigmaTranslational, float sigmaRotational)
{
    AddPoseConstraint(fromNode, toNode, 
                        x, y, z, roll, pitch, yaw,
                        sigmaTranslational, sigmaTranslational, sigmaTranslational,
                        sigmaRotational, sigmaRotational, sigmaRotational);
}


void anloro::WorldModelInterface::AddPoseConstraint(int fromNode, int toNode, 
                                                    Eigen::Affine3f affineT,
                                                    float sigmaX, float sigmaY, float sigmaZ, 
                                                    float sigmaRoll, float sigmaPitch, float sigmaYaw)
{
    Transform transform(affineT);
    AddPoseConstraint(fromNode, toNode,
                      transform,
                      sigmaX, sigmaY, sigmaZ,
                      sigmaRoll, sigmaPitch, sigmaYaw);
}

void anloro::WorldModelInterface::AddPoseConstraint(int fromNode, int toNode, 
                                                    Eigen::Affine3f affineT, 
                                                    float sigmaTranslational, float sigmaRotational)
{
    AddPoseConstraint(fromNode, toNode,
                      affineT,
                      sigmaTranslational, sigmaTranslational, sigmaTranslational,
                      sigmaRotational, sigmaRotational, sigmaRotational);
}

void anloro::WorldModelInterface::AddPoseConstraint(int fromNode, int toNode, 
                                                    Eigen::Affine3f affineT, Eigen::Matrix<float, 6, 6> noiseModel)
{
    Transform transform(affineT);

    float sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
    sigmaX = noiseModel(0,0);
    sigmaY = noiseModel(1,1);
    sigmaZ = noiseModel(2,2);
    sigmaRoll = noiseModel(3,3);
    sigmaPitch = noiseModel(4,4);
    sigmaYaw = noiseModel(5,5);

    AddPoseConstraint(fromNode, toNode,
                      transform,
                      sigmaX, sigmaY, sigmaZ,
                      sigmaRoll, sigmaPitch, sigmaYaw);
}

void anloro::WorldModelInterface::AddPoseConstraint(int fromNode, int toNode,
                                                    Transform transform,
                                                    float sigmaX, float sigmaY, float sigmaZ,
                                                    float sigmaRoll, float sigmaPitch, float sigmaYaw)
{
    MsgUdp newMsg;
    newMsg.type = 'd';
    float x, y, z, pitch, yaw, roll; 
    transform.GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
    newMsg.element.poseFactor.pose.x = x;
    newMsg.element.poseFactor.pose.y = y;
    newMsg.element.poseFactor.pose.z = z;
    newMsg.element.poseFactor.pose.roll = roll;
    newMsg.element.poseFactor.pose.pitch = pitch;
    newMsg.element.poseFactor.pose.yaw = yaw;
    newMsg.element.poseFactor.idFrom = fromNode;
    newMsg.element.poseFactor.idTo = toNode;
    newMsg.element.poseFactor.unc.sigmaX = sigmaX;
    newMsg.element.poseFactor.unc.sigmaY = sigmaY;
    newMsg.element.poseFactor.unc.sigmaZ = sigmaZ;
    newMsg.element.poseFactor.unc.sigmaRoll = sigmaRoll;
    newMsg.element.poseFactor.unc.sigmaPitch = sigmaPitch;
    newMsg.element.poseFactor.unc.sigmaYaw = sigmaYaw;
    client.Send(newMsg);
}

// Get the interface's unique ID
std::string anloro::WorldModelInterface::GetInterfaceId()
{

    return _uniqueID;
}

// Get the optimized poses from the World model
std::map<int, Eigen::Affine3f> anloro::WorldModelInterface::GetOptimizedPoses()
{
    typedef std::pair<int, Eigen::Affine3f> optimizedPose;

    // Send request for the optimized poses
    MsgUdp newMsg;
    newMsg.type = 'f';
    client.Send(newMsg);

    int recvlen;
    // Create the buffer
    void * buffer = calloc(1, sizeof(struct MsgUdp));

    recvlen = client.Receive(buffer);
    MsgUdp * msg = (struct MsgUdp *)buffer;
    std::cout << "INFO: Graph received with size: " << msg->size << std::endl;
    
    // std::cout << "INFO: Waiting for size" << std::endl;
    // // Receive data using the buffer
    // recvlen = client.Receive(buffer);
    // // Interpret the data using the message structure
    // MsgUdp * msg = (struct MsgUdp *)buffer;
    // // Receive the lenght of the graph
    // int size = msg->size;
    // std::cout << "INFO: Size received: " << size << std::endl;
    // Prepare to receive the graph
    // std::vector<KeyFrameData> graph(size);

    std::map<int, Eigen::Affine3f> mappedOptimizedPoses;
    float x, y, z, pitch, yaw, roll;
    int id;
    for (int i = 0; i < msg->size; i++)
    {
        // // void * buffer = calloc(1, sizeof(struct MsgUdp));
        // recvlen = client.Receive(buffer);
        // // Interpret the data using the message structure
        // MsgUdp * msg = (struct MsgUdp *)buffer;
        // // graph[i] = msg->element.keyFrame;
        // std::cout << "INFO: Received element KeyFrame: " << msg->element.keyFrame.id << std::endl;
        // // std::cout << "INFO: Receiving i: " << i << std::endl;

        // id = msg->element.keyFrame.id;
        // x = msg->element.keyFrame.pose.x;
        // y = msg->element.keyFrame.pose.y;
        // z = msg->element.keyFrame.pose.z;
        // roll = msg->element.keyFrame.pose.roll;
        // pitch = msg->element.keyFrame.pose.pitch;
        // yaw = msg->element.keyFrame.pose.yaw;

        id = msg->element.graph.at(i).id;
        x = msg->element.graph.at(i).pose.x;
        y = msg->element.graph.at(i).pose.y;
        z = msg->element.graph.at(i).pose.z;
        roll = msg->element.graph.at(i).pose.roll;
        pitch = msg->element.graph.at(i).pose.pitch;
        yaw = msg->element.graph.at(i).pose.yaw;

        // std::cout << "INFO: Processed received pose with ID: " << id << std::endl;

        Transform t = Transform(x, y, z, roll, pitch, yaw);   
        mappedOptimizedPoses.insert(optimizedPose(id, t.GetAffineTransform()));

    }

    // std::map<int, Eigen::Affine3f> mappedOptimizedPoses;
    // float x, y, z, pitch, yaw, roll;
    // int id;
    // for (int i = 0; i < size; i++)
    // {
    //     id = graph.at(i).id;
    //     x = graph.at(i).pose.x;
    //     y = graph.at(i).pose.y;
    //     z = graph.at(i).pose.z;
    //     roll = graph.at(i).pose.roll;
    //     pitch = graph.at(i).pose.pitch;
    //     yaw = graph.at(i).pose.yaw;
    //     Transform t = Transform(x, y, z, roll, pitch, yaw);   
    //     mappedOptimizedPoses.insert(optimizedPose(id, t.GetAffineTransform()));
    // }

    return mappedOptimizedPoses;
}

// Call the UndoOdometryCorrection function
// void anloro::WorldModelInterface::UndoOdometryCorrection(int lastLoopId, Eigen::Matrix4f uncorrection)
// {
//     int id = NodeInternalMapId(lastLoopId);
//     _worldModel->UndoOdometryCorrection(id, uncorrection);
// }

// Call the optimizer for the World model
void anloro::WorldModelInterface::Optimize()
{
    MsgUdp newMsg;
    newMsg.type = 'g';
    client.Send(newMsg);
}

// Save the poses into a txt file in Euler format

void anloro::WorldModelInterface::SavePosesRaw()
{
    MsgUdp newMsg;
    newMsg.type = 'e';
    client.Send(newMsg);
}

void anloro::WorldModelInterface::SavePosesRaw(std::string name)
{
    MsgUdp newMsg;
    newMsg.type = 'e';
    client.Send(newMsg);
}

// This may be removed after the testing and automatically set with an odometru manager in the world model
// void anloro::WorldModelInterface::SetCurrentState(Transform t)
// {
//     _worldModel->currentState = t;
// }