/**
 * @file   KeyFrame.cpp
 * @brief  Defines the Key-Frame Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#include "RefFrame.cpp"

struct null_type {};

// Add more typenames if needed now we have 3 (i.e Image, LiDAR, Features)
class KeyFrame : RefFrame
{
    public:
        // Constructors
        KeyFrame(double x, double y, double z, double roll, double pitch, double yaw);
        template <class T> 
        KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, T data);
        template <class T, class U> 
        KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, T data1, U data2);
        template <class T, class U, class V> 
        KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, T data1, U data2, V data3);

        template <class T = null_type, class U = null_type, class V = null_type>
        struct RawData
        {
            // (Image, LiDAR, Features)
            T *SensorData1; 
            U *SensorData2; 
            V *SensorData3;
        };

    protected:
        RawData <class T, class U, class V> _rawData;
};

KeyFrame::KeyFrame(double x, double y, double z, double roll, double pitch, double yaw)
{
    _x = x;
    _y = y;
    _z = z;
    _roll = roll;
    _pitch = pitch;
    _yaw = yaw;
}

template <class T> 
KeyFrame::KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, T data)
{
    _x = x;
    _y = y;
    _z = z;
    _roll = roll;
    _pitch = pitch;
    _yaw = yaw;
    _rawData.SensorData1 = data;
}

template <class T, class U> 
KeyFrame::KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, T data1, U data2)
{
    _x = x;
    _y = y;
    _z = z;
    _roll = roll;
    _pitch = pitch;
    _yaw = yaw;
    _rawData.SensorData1 = data1;
    _rawData.SensorData2 = data2;
}

template <class T, class U, class V> 
KeyFrame::KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, T data1, U data2, V data3)
{
    _x = x;
    _y = y;
    _z = z;
    _roll = roll;
    _pitch = pitch;
    _yaw = yaw;
    _rawData.SensorData1 = data1;
    _rawData.SensorData2 = data2;
    _rawData.SensorData3 = data3;
}
