/**
 * @file   KeyFrame.h
 * @brief  Defines the Key-Frame Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#pragma once

#include "RefFrame.h"
#include <tuple>


namespace anloro{

template <typename... Ts>
class KeyFrame : public RefFrame
{
public:
    // Constructors
    KeyFrame(double x, double y, double z, double roll, double pitch, double yaw);
    KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, Ts &...t);
    // The compiler takes care of the default constructor
    KeyFrame() = default;

    template <typename T>
    T GetData();

protected:
    std::tuple<Ts...> _rawData;
};

template <class... Ts>
KeyFrame<Ts...>::KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, Ts &...data)
{
    _x = x;
    _y = y;
    _z = z;
    _roll = roll;
    _pitch = pitch;
    _yaw = yaw;
    std::tuple<Ts...> rawData = std::tuple(data...);
    _rawData = rawData;
}

template <typename... Ts>
template <typename T>
T KeyFrame<Ts...>::GetData()
{
    T element = std::get<T>(_rawData);
    return element;
}

} // namespace anloro
