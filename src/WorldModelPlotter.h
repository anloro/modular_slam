/**
 * @file   WorldModelPlotter.h
 * @brief  World model graphical output.
 * @author Ángel Lorente Rogel
 * @date   18/05/2021
 */

#pragma once

#include <SFML/Graphics.hpp>


namespace anloro{

class WorldModelPlotter
{
    public:
        WorldModelPlotter();
        ~WorldModelPlotter();
        void Spin();
        void SetAxis(std::vector<float> *xAxis, std::vector<float> *yAxis);
        std::vector<float> *GetxAxis();
        std::vector<float> *GetyAxis();
        sf::RenderWindow *GetWindow();
        void SaveImage(const char *filename);

    protected:
        sf::RenderWindow * _window;
        sf::Font _font;
        std::vector<float> * _xAxis;
        std::vector<float> * _yAxis;
};


} // namespace anloro
