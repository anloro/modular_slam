/**
 * @file   LandMark.cpp
 * @brief  Defines the LandMark Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#include "LandMark.h"
#include <tuple>

LandMark::LandMark(float x, float y, float z)
{
    LandMark::_x = x;
    LandMark::_y = y;
    LandMark::_z = z;
}

void LandMark::SetTranslationalVector(float x, float y, float z)
{
    LandMark::_x = x;
    LandMark::_y = y;
    LandMark::_z = z;
}

std::tuple<float, float, float> LandMark::GetTranslationalVector()
{
    return std::tuple<float, float, float>{_x, _y, _z};
}

// int main(){
//     LandMark newLM = LandMark(0, 0, 0);
//     std::tuple tLM = newLM.GetTranslationalVector();
//     float xLM = std::get<0>(tLM);
//     float yLM = std::get<1>(tLM);
//     float zLM = std::get<2>(tLM);
//     std::cout << "My LM translational vector: (" << xLM << ", " << yLM << ", " << zLM << ")" << std::endl;
//     return 0;
// }