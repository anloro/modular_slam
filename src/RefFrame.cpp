/**
 * @file   RefFrame.cpp
 * @brief  Defines the Reference Frame Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */
#include "RefFrame.h"


anloro::RefFrame::RefFrame(Transform transform)
{
    _transform = transform;
}

void anloro::RefFrame::SetTransform(Transform transform)
{
    _transform = transform;
}
