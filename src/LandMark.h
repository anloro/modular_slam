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
// I will have to mofify this to have an id to the node it is related to,
//  it has to inherit from refence frame class and allow storing features 
//  and also we have to make it to work with gtsam as an extra constraint.
// Or maybe put it directly as a field in a keyframe.
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