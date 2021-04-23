// /**
//  * @file   KeyFrame.cpp
//  * @brief  Defines the Key-Frame Entity.
//  * @author Ángel Lorente Rogel
//  * @date   19/04/2021
//  */

// #include "KeyFrame.h"

// template<>
// KeyFrame<int>::KeyFrame(double x, double y, double z, double roll, double pitch, double yaw)
// {
//     _x = x;
//     _y = y;
//     _z = z;
//     _roll = roll;
//     _pitch = pitch;
//     _yaw = yaw;
// }

// template <class... Ts>
// KeyFrame<Ts...>::KeyFrame(double x, double y, double z, double roll, double pitch, double yaw, Ts &...data)
// {
//     _x = x;
//     _y = y;
//     _z = z;
//     _roll = roll;
//     _pitch = pitch;
//     _yaw = yaw;
//     std::tuple<Ts...> rawData = std::tuple(data...);
//     _rawData = rawData;

// }

// template <typename... Ts>
// template <typename T>
// T KeyFrame<Ts...>::GetData()
// {
//     T element = std::get<T>(_rawData);
//     return element;

// }

// // int main()
// // {
// //     LandMark newLM = LandMark(0, 0, 0);
// //     std::tuple tLM = newLM.GetTranslationalVector();
// //     double xLM = std::get<0>(tLM);
// //     double yLM = std::get<1>(tLM);
// //     double zLM = std::get<2>(tLM);
// //     std::cout << "My LM translational vector: (" << xLM << ", " << yLM << ", " << zLM << ")" << std::endl;

// //     RefFrame newRF = RefFrame(0, 0, 0, 0, 0, 0);
// //     std::tuple tRF = newRF.GetTranslationalVector();
// //     double xRF = std::get<0>(tRF);
// //     double yRF = std::get<1>(tRF);
// //     double zRF = std::get<2>(tRF);
// //     std::cout << "My RF translational vector: (" << xRF << ", " << yRF << ", " << zRF << ")" << std::endl;
// //     std::tuple rRF = newRF.GetRotationalVector();
// //     double rollRF = std::get<0>(rRF);
// //     double pitchRF = std::get<1>(rRF);
// //     double yawRF = std::get<2>(rRF);
// //     std::cout << "My RF rotational vector: (" << rollRF << ", " << pitchRF << ", " << yawRF << ")" << std::endl;

// //     struct mystructure
// //     {
// //         double a = 0;
// //         int b = 23;
// //         char c = 'c';
// //     }thisstruct;

// //     KeyFrame<mystructure> newKF = KeyFrame<mystructure>(0, 0, 0, 0, 0, 0, thisstruct);
// //     mystructure myObject = newKF.GetData<mystructure>();
// //     int myb = myObject.b;
// //     std::cout << "I stored this inside the KF: (" << myb << ")" << std::endl;
// //     std::tuple tKF = newKF.GetTranslationalVector();
// //     double xKF = std::get<0>(tKF);
// //     double yKF = std::get<1>(tKF);
// //     double zKF = std::get<2>(tKF);
// //     std::cout << "My KF translational vector: (" << xKF << ", " << yKF << ", " << zKF << ")" << std::endl;
// //     std::tuple rKF = newKF.GetRotationalVector();
// //     double rollKF = std::get<0>(rKF);
// //     double pitchKF = std::get<1>(rKF);
// //     double yawKF = std::get<2>(rKF);
// //     std::cout << "My KF rotational vector: (" << rollKF << ", " << pitchKF << ", " << yawKF << ")" << std::endl;

// //     return 0;
// // }