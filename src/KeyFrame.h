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
    KeyFrame(double timeStamp, Transform transform, Ts &...t);
    // The compiler takes care of the default constructor
    KeyFrame() = default;

    template <typename T>
    T GetData();
    double GetTimeStamp(){return _timeStamp;};

protected:
    std::tuple<Ts...> _rawData;
    double _timeStamp;
};

template <class... Ts>
KeyFrame<Ts...>::KeyFrame(double timeStamp, Transform transform, Ts &...data)
{
    _timeStamp = timeStamp;
    _transform = transform;

    std::tuple<Ts...> rawData = std::tuple<Ts...>(data...);
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
