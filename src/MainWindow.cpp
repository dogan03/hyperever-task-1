#include "MainWindow.h"
#include <string>

MainWindow::MainWindow()
    : width(WINDOW_WIDTH)
    , height(WINDOW_HEIGHT)
    , window(sf::VideoMode(800, 600), "Hyperever Task 1")
    , background_color(sf::Color::Black)
    , cartpole(CartPoleParams())
{
    initializeWindow();
    initializeFont();
    initializeShapes();
    initializeTexts();
}

void MainWindow::initializeWindow() {
    window.setFramerateLimit(FRAME_RATE);
}

void MainWindow::initializeFont() {
    if (!font.loadFromFile("include/arial.ttf")) {
        throw std::runtime_error("Failed to load font arial.ttf");
    }
}

void MainWindow::initializeShapes() {
    simulation_window.setSize(sf::Vector2f(SIM_AREA_WIDTH, SIM_AREA_HEIGHT));
    centerShape(simulation_window, width / 2.f, height / 2.f);
    simulation_window.setFillColor(sf::Color::Transparent);
    simulation_window.setOutlineThickness(OUTLINE_THICKNESS);
    simulation_window.setOutlineColor(sf::Color::White);

    road.setSize(sf::Vector2f(ROAD_WIDTH, ROAD_HEIGHT));
    centerShape(road, width / 2.f, height / 2.f + 20.f);
    road.setFillColor(sf::Color::Transparent);
    road.setOutlineThickness(OUTLINE_THICKNESS);
    road.setOutlineColor(sf::Color::White);

    cart.setSize(sf::Vector2f(CART_WIDTH, CART_HEIGHT));
    centerShape(cart, width / 2.f, height / 2.f + 20.f);
    cart.setFillColor(sf::Color::Red);
    cart.setOutlineThickness(OUTLINE_THICKNESS);
    cart.setOutlineColor(sf::Color::White);
}

void MainWindow::initializeTexts() {
    setupText(title, "Hyperever Simulation Area", width / 2.f, 20.f, 30);
    setupText(tick1, "-250", 100.f, 350.f);
    setupText(tick2, "+250", 700.f, 350.f);
    setupText(theta_txt, "", 400.f, 550.f);
    setupText(x_txt, "", 300.f, 550.f);
}

void MainWindow::setupText(sf::Text& text, const std::string& content, float x, float y, unsigned int size) {
    text.setFont(font);
    text.setString(content);
    text.setCharacterSize(size);
    text.setFillColor(sf::Color::White);

    sf::FloatRect bounds = text.getLocalBounds();
    text.setOrigin(bounds.width / 2.f, bounds.height / 2.f);
    text.setPosition(x, y);
}

void MainWindow::centerShape(sf::RectangleShape& shape, float x, float y) {
    sf::FloatRect bounds = shape.getLocalBounds();
    shape.setOrigin(bounds.width / 2.f, bounds.height / 2.f);
    shape.setPosition(x, y);
}

void MainWindow::drawTitle() {
    window.draw(title);
}

void MainWindow::drawSimulationWindow() {
    window.draw(simulation_window);
}

void MainWindow::drawRoad() {
    window.draw(road);
}

void MainWindow::drawCart() {
    cart.setPosition(width / 2.f + static_cast<float>(cartpole.x), height / 2.f + 20.f);
    window.draw(cart);
}

void MainWindow::drawPole() {
    float pivot_x = width / 2.f + static_cast<float>(cartpole.x);
    float pivot_y = height / 2.f + 20.f;

    sf::RectangleShape pole(sf::Vector2f(POLE_WIDTH, POLE_VISUAL_LENGTH));
    pole.setFillColor(sf::Color::Magenta);
    pole.setOrigin(POLE_WIDTH / 2.f, POLE_VISUAL_LENGTH);

    float angle_degrees = static_cast<float>(cartpole.theta * 180.0 / M_PI);
    pole.setRotation(angle_degrees);
    pole.setPosition(pivot_x, pivot_y);

    window.draw(pole);
}

void MainWindow::drawTicks() {
    window.draw(tick1);
    window.draw(tick2);
}

void MainWindow::drawTheta() {
    double theta_degrees = cartpole.theta * 180.0 / M_PI;
    std::string theta_str = std::to_string(theta_degrees);
    
    size_t decimal_pos = theta_str.find('.');
    if (decimal_pos != std::string::npos && decimal_pos + 3 < theta_str.length()) {
        theta_str = theta_str.substr(0, decimal_pos + 3);
    }
    
    theta_txt.setString("Theta: " + theta_str);
    window.draw(theta_txt);
}

void MainWindow::drawX() {
    std::string x_str = std::to_string(cartpole.x);
    
    size_t decimal_pos = x_str.find('.');
    if (decimal_pos != std::string::npos && decimal_pos + 3 < x_str.length()) {
        x_str = x_str.substr(0, decimal_pos + 3);
    }
    
    x_txt.setString("X: " + x_str);
    window.draw(x_txt);
}