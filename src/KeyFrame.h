/**
 * @file   KeyFrame.h
 * @brief  Defines the Key-Frame Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#pragma once

#include "RefFrame.h"
#include <tuple>

// Add more typenames if needed now we have 3 (i.e Image, LiDAR, Features)
template <typename... Ts>
class KeyFrame : public RefFrame
{
public:
    // Constructors
    KeyFrame(double x, double y, double z, double roll, double pitch, double yaw);
    // template <typename... Ts>
    KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, Ts &...t);

    std::tuple<Ts...> GetData();
    // // template <class T>
    // // KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, Traits data);
    // // template <class T, class U>
    // // KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, T data1, U data2);
    // // // template <class T, class U, class V>
    // // KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, T data1, U data2, V data3);
    // template <typename... Ts>
    // struct Traits
    // {
    //     using Tuple = std::tuple<Ts...>;
    //     static constexpr auto Size = sizeof...(Ts);
    //     // template <std::size_t N>
    //     // using Nth = typename std::tuple_element<N, Tuple>::type;
    //     // using First = Nth<0>;
    //     // using Last = Nth<Size - 1>;
    // };
    // template <typename... Ts>
    // void mydata(Ts&... t);
    // // struct RawData
    // // {
    // //     // (Image, LiDAR, Features)
    // //     T *SensorData1;
    // //     U *SensorData2;
    // //     V *SensorData3;
    // // };

protected:
    std::tuple<Ts...> _rawData;
};