/**
 * @file   RefFrame.cpp
 * @brief  Defines the Reference Frame Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */
#include "RefFrame.h"


anloro::RefFrame::RefFrame(double x, double y, double z, double roll, double pitch, double yaw)
{
    _x = x;
    _y = y;
    _z = z;
    _roll = roll;
    _pitch = pitch;
    _yaw = yaw;
}

void anloro::RefFrame::SetTranslationalVector(double x, double y, double z)
{
    RefFrame::_x = x;
    RefFrame::_y = y;
    RefFrame::_z = z;
}

std::tuple<double, double, double> anloro::RefFrame::GetTranslationalVector()
{
    return std::tuple<double, double, double>{_x, _y, _z};
}

void anloro::RefFrame::SetRotationalVector(double roll, double pitch, double yaw)
{
    RefFrame::_roll = roll;
    RefFrame::_pitch = pitch;
    RefFrame::_yaw = yaw;
}

std::tuple<double, double, double> anloro::RefFrame::GetRotationalVector()
{
    return std::tuple<double, double, double>{_roll, _pitch, _yaw};
}

void anloro::RefFrame::GetTranslationalAndEulerAngles(double &x, double &y, double &z, double &roll, double &pitch, double &yaw)
{
    x = RefFrame::_x;
    y = RefFrame::_y;
    z = RefFrame::_z;
    roll = RefFrame::_roll;
    pitch = RefFrame::_pitch;
    yaw = RefFrame::_yaw;
}

// int main()
// {
// //     LandMark newLM = LandMark(0, 0, 0);
// //     std::tuple tLM = newLM.GetTranslationalVector();
// //     double xLM = std::get<0>(tLM);
// //     double yLM = std::get<1>(tLM);
// //     double zLM = std::get<2>(tLM);
// //     std::cout << "My LM translational vector: (" << xLM << ", " << yLM << ", " << zLM << ")" << std::endl;

//     RefFrame newRF = RefFrame(0, 0, 30, 50, 0, 0);
//     double x, y, z, roll, pitch, yaw;
//     newRF.GetTranslationalAndEulerAngles(x, y, z, roll, pitch, yaw);
//     std::cout << "My ref frame coordinates: (" << x << ", " << y << ", " << z << roll << ", " << pitch << ", " << yaw << ")" << std::endl;

//     //     std::tuple tRF = newRF.GetTranslationalVector();
//     //     double xRF = std::get<0>(tRF);
//     //     double yRF = std::get<1>(tRF);
//     //     double zRF = std::get<2>(tRF);
//     //     std::cout << "My RF translational vector: (" << xRF << ", " << yRF << ", " << zRF << ")" << std::endl;
//     //     std::tuple rRF = newRF.GetRotationalVector();
//     //     double rollRF = std::get<0>(rRF);
//     //     double pitchRF = std::get<1>(rRF);
//     //     double yawRF = std::get<2>(rRF);
//     //     std::cout << "My RF rotational vector: (" << rollRF << ", " << pitchRF << ", " << yawRF << ")" << std::endl;

//     return 0;

// }