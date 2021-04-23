/**
 * @file   KeyFrame.h
 * @brief  Defines the Key-Frame Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#pragma once

#include "RefFrame.h"
#include <tuple>


template <typename... Ts>
class KeyFrame : public RefFrame
{
public:
    // Constructors
    KeyFrame(double x, double y, double z, double roll, double pitch, double yaw);
    KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, Ts &...t);

    template <typename T>
    T GetData();

protected:
    std::tuple<Ts...> _rawData;
};