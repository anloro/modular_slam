/**
 * @file   LandMark.cpp
 * @brief  Defines the LandMark Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#include "LandMark.h"
#include <tuple>

LandMark::LandMark(double x, double y, double z)
{
    LandMark::_x = x;
    LandMark::_y = y;
    LandMark::_z = z;
}

void LandMark::SetTranslationalVector(double x, double y, double z)
{
    LandMark::_x = x;
    LandMark::_y = y;
    LandMark::_z = z;
}

std::tuple<double, double, double> LandMark::GetTranslationalVector()
{
    return std::tuple<double, double, double>{_x, _y, _z};
}

// int main(){
//     LandMark newLM = LandMark(0, 0, 0);
//     std::tuple tLM = newLM.GetTranslationalVector();
//     double xLM = std::get<0>(tLM);
//     double yLM = std::get<1>(tLM);
//     double zLM = std::get<2>(tLM);
//     std::cout << "My LM translational vector: (" << xLM << ", " << yLM << ", " << zLM << ")" << std::endl;
//     return 0;
// }