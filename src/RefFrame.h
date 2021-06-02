/**
 * @file   RefFrame.h
 * @brief  Defines the Reference Frame Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#pragma once

#include <tuple>
#include <iostream>
#include "Transform.h"

namespace anloro{

class RefFrame
{
public:
    // Constructors
    RefFrame(Transform transform);
    // The compiler takes care of the default constructor
    RefFrame() = default;

    // Member functions
    void SetTransform(Transform transform);
    Transform GetTransform(){return _transform;};

protected:
    Transform _transform;
};

} // namespace anloro