/**
 * @file   RefFrame.cpp
 * @brief  Defines the Reference Frame Entity.
 * @author Ángel Lorente Rogel
 * @date   19/04/2021
 */

#include "LandMark.cpp"

class RefFrame : LandMark
{
    public:
        // Constructors
        RefFrame(double x, double y, double z, double roll, double pitch, double yaw);
        // Member functions
        void SetRotationalVector(double x, double y, double z);
        std::tuple<double, double, double> GetRotationalVector();

    protected:
        double _roll, _pitch, _yaw;
};

RefFrame::RefFrame(double x, double y, double z, double roll, double pitch, double yaw)
{
    _x = x;
    _y = y;
    _z = z;
    _roll = roll;
    _pitch = pitch;
    _yaw = yaw;
}

void RefFrame::SetRotationalVector(double roll, double pitch, double yaw)
{
    RefFrame::_roll = roll;
    RefFrame::_pitch = pitch;
    RefFrame::_yaw = yaw;
}

std::tuple<double, double, double> RefFrame::GetRotationalVector()
{
    return std::tuple<double, double, double>{_roll, _pitch, _yaw};
}