/**
 * @file   WorldModelPlotter.cpp
 * @brief  World model graphical output.
 * @author Ángel Lorente Rogel
 * @date   18/05/2021
 */
#include "WorldModelPlotter.h"
#include <iostream>
#include <thread>

#include "SFPlot.h"

anloro::WorldModelPlotter::WorldModelPlotter()
{
    sf::ContextSettings settings;
    settings.antialiasingLevel = 4;

    _window = new sf::RenderWindow(sf::VideoMode(800, 800), "Trajectory", sf::Style::Default, settings);
    _font.loadFromFile("/usr/local/share/fonts/Lato-Regular.ttf");
    // _font.loadFromFile("../lato/Lato-Regular.ttf");

    _xAxis = new std::vector<float>{};
    _yAxis = new std::vector<float>{};
}

std::vector<float> *anloro::WorldModelPlotter::GetxAxis()
{
    return _xAxis;
}

std::vector<float> *anloro::WorldModelPlotter::GetyAxis()
{
    return _yAxis;
}

sf::RenderWindow* anloro::WorldModelPlotter::GetWindow()
{
    return _window;
}

void anloro::WorldModelPlotter::SetAxis(std::vector<float> *xAxis, std::vector<float> *yAxis)
{
    delete _xAxis;
    delete _yAxis;
    _xAxis = xAxis;
    _yAxis = yAxis;
}

void anloro::WorldModelPlotter::Spin()
{
    while (_window->isOpen())
    {

        _window->clear(sf::Color(30, 30, 30));

        // Update the plot
        if (_xAxis->size() > 2 && _yAxis->size() > 2){
            csrc::SFPlot plot(sf::Vector2f(0, 0), sf::Vector2f(800, 800), 50, _font, "X Axis", "Y Axis");

            float xmin = *min_element(_xAxis->begin(), _xAxis->end());
            float xmax = *max_element(_xAxis->begin(), _xAxis->end());
            float ymin = *min_element(_yAxis->begin(), _yAxis->end());
            float ymax = *max_element(_yAxis->begin(), _yAxis->end());
            float xstep = (xmax - xmin) / 10;
            float ystep = (ymax - ymin) / 10;

            float amax = std::max(xmax, ymax);
            float amin = std::min(xmin, ymin);

            std::vector<float> x = {};
            for (float i : *_xAxis){
                x.push_back(i - xmin);
            }
            std::vector<float> y = {};
            for (float i : *_yAxis)
            {
                y.push_back(i - ymin);
            }

            // xmin = *min_element(x.begin(), x.end());
            // xmax = *max_element(x.begin(), x.end());
            // ymin = *min_element(y.begin(), y.end());
            // ymax = *max_element(y.begin(), y.end());
            // xstep = (xmax - xmin) / 10;
            // ystep = (ymax - ymin) / 10;

            // amax = std::max(xmax, ymax);
            // amin = std::min(xmin, ymin);

            // csrc::PlotDataSet set(*_xAxis, *_yAxis, sf::Color::White, "Trajectory", csrc::PlottingType::LINE);
            csrc::PlotDataSet set(x, y, sf::Color::White, "Trajectory", csrc::PlottingType::LINE);
            plot.AddDataSet(set);

            // plot.SetupAxes(amin, amax, amin, amax, xstep, ystep, sf::Color::White);
            plot.SetupAxes();
            plot.GenerateVertices();

            _window->draw(plot);
        }

        _window->display();

        // Handle the close of the window
        sf::Event event;
        if (_window->pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                _window->close();
            }
        }

        using namespace std::literals::chrono_literals;
        std::this_thread::sleep_for(1s);
    }
}

void anloro::WorldModelPlotter::SaveImage(const char * filename)
{
    sf::Texture texture;
    texture.create(_window->getSize().x, _window->getSize().y);
    texture.update(*_window);
    if (texture.copyToImage().saveToFile(filename))
    {
        std::cout << "screenshot saved to " << filename << std::endl;
    }
    else
    {
        // Error handling
    }
}

// Destructor
anloro::WorldModelPlotter::~WorldModelPlotter()
{
    delete _xAxis;
    delete _yAxis;
    delete _window;
}