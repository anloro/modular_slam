/**
 * @file   LandMark.cpp
 * @brief  Defines the LandMark Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#include <tuple>
#include <iostream>

class Entity{};

// Entity::Entity(){}

class LandMark : Entity{
    public:
        // Constructors
        LandMark(double x, double y, double z);
        // The compiler takes care of the default constructor
        LandMark() = default;

        // Member functions
        void SetTranslationalVector(double x, double y, double z);
        std::tuple<double, double, double> GetTranslationalVector();

    protected:
        double _x,_y,_z;
};

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
