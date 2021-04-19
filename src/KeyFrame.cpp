/**
 * @file   KeyFrame.cpp
 * @brief  Defines the Key-Frame Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#include "RefFrame.cpp"

template <class T>
class KeyFrame: RefFrame
{
    public:
        // Constructors
        KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, T data);

        struct RawData{
            T *SensorData;
        };

    protected:
        RawData _rawData;
};

template <class T>
KeyFrame<T>::KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, T data)
{
    _x = x;
    _y = y;
    _z = z;
    _roll = roll;
    _pitch = pitch;
    _yaw = yaw;
    _rawData.SensorData = data;
}