/**
 * @file   LandMark.h
 * @brief  Defines the LandMark Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#include <tuple>
#include <iostream>

// class mySerializableEntity{};

// Entity::Entity(){}

class LandMark
{
public:
    // Constructors
    LandMark(double x, double y, double z);
    // The compiler takes care of the default constructor
    LandMark() = default;

    // Member functions
    void SetTranslationalVector(double x, double y, double z);
    std::tuple<double, double, double> GetTranslationalVector();

protected:
    double _x, _y, _z;
};