#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <SFML/Graphics.hpp>
#include "CartPole.h"

/**
 * @brief SFML-based visualization window for CartPole simulation
 * 
 * Provides real-time graphical display of the cart-pole system including:
 * - Cart and pole rendering with accurate position and angle
 * - Simulation boundary markers
 * - Real-time state display (position, angle)
 * - Clean visual interface with labeled components
 * 
 * The window uses a fixed coordinate system where the center represents
 * the origin (x=0) of the CartPole track. Visual elements are scaled
 * appropriately for clear visualization.
 */
class MainWindow {
private:
    static constexpr int WINDOW_WIDTH = 800;              ///< Window width in pixels
    static constexpr int WINDOW_HEIGHT = 600;             ///< Window height in pixels
    static constexpr int FRAME_RATE = 60;                 ///< Target frame rate (Hz)
    static constexpr float SIM_AREA_WIDTH = 700.f;        ///< Simulation area width (pixels)
    static constexpr float SIM_AREA_HEIGHT = 450.f;       ///< Simulation area height (pixels)
    static constexpr float ROAD_WIDTH = 600.f;            ///< Track/road width (pixels)
    static constexpr float ROAD_HEIGHT = 20.f;            ///< Track/road height (pixels)
    static constexpr float CART_WIDTH = 100.f;            ///< Cart visual width (pixels)
    static constexpr float CART_HEIGHT = 20.f;            ///< Cart visual height (pixels)
    static constexpr float POLE_WIDTH = 4.f;              ///< Pole visual width (pixels)
    static constexpr float POLE_VISUAL_LENGTH = 180.f;    ///< Pole visual length (pixels)
    static constexpr float OUTLINE_THICKNESS = 3.f;       ///< Border thickness for shapes (pixels)

    sf::Font font;                     ///< Font for text rendering
    sf::RectangleShape simulation_window; ///< Simulation boundary rectangle
    sf::RectangleShape road;           ///< Track/road visual element
    sf::RectangleShape cart;           ///< Cart visual representation
    sf::Text title;                    ///< Title text "Hyperever Simulation Area"
    sf::Text tick1;                    ///< Left boundary marker (-250)
    sf::Text tick2;                    ///< Right boundary marker (+250)
    sf::Text theta_txt;                ///< Pole angle display
    sf::Text x_txt;                    ///< Cart position display

    /**
     * @brief Configures window properties (frame rate, etc.)
     */
    void initializeWindow();
    
    /**
     * @brief Loads font from arial.ttf file
     * @throws std::runtime_error if font file cannot be loaded
     */
    void initializeFont();
    
    /**
     * @brief Creates and positions all geometric shapes
     * 
     * Initializes simulation window border, road/track, and cart
     * with appropriate sizes, colors, and positions.
     */
    void initializeShapes();
    
    /**
     * @brief Creates and positions all text elements
     * 
     * Sets up title, boundary markers, and state display texts
     * with appropriate fonts, sizes, and positions.
     */
    void initializeTexts();
    
    /**
     * @brief Helper to configure text properties
     * @param text Reference to text object to configure
     * @param content String content to display
     * @param x Horizontal position (pixels)
     * @param y Vertical position (pixels)
     * @param size Character size (default 20)
     * 
     * Sets font, content, size, color, and centers the text at position.
     */
    void setupText(sf::Text& text, const std::string& content, float x, float y, unsigned int size = 20);
    
    /**
     * @brief Helper to center a shape at given position
     * @param shape Reference to shape to center
     * @param x Horizontal center position (pixels)
     * @param y Vertical center position (pixels)
     * 
     * Sets shape origin to its center and positions it at (x, y).
     */
    void centerShape(sf::RectangleShape& shape, float x, float y);

public:
    sf::RenderWindow window;        ///< SFML render window (public for event handling)
    sf::Color background_color;     ///< Background color (black)
    CartPole cartpole;              ///< CartPole physics object being visualized
    int width;                      ///< Window width (pixels)
    int height;                     ///< Window height (pixels)

    /**
     * @brief Constructs and initializes the visualization window
     * 
     * Creates SFML window, loads font, initializes all visual elements,
     * and sets up the CartPole physics object.
     * 
     * @throws std::runtime_error if font loading fails
     */
    MainWindow();

    /**
     * @brief Draws simulation title text
     */
    void drawTitle();
    
    /**
     * @brief Draws simulation area boundary rectangle
     */
    void drawSimulationWindow();
    
    /**
     * @brief Draws track/road element
     */
    void drawRoad();
    
    /**
     * @brief Draws cart at current position
     * 
     * Updates cart position based on cartpole.x and renders it.
     */
    void drawCart();
    
    /**
     * @brief Draws pole at current angle
     * 
     * Creates and rotates pole visual based on cartpole.theta,
     * positioned at cart location. Angle is converted from radians
     * to degrees for SFML rotation.
     */
    void drawPole();
    
    /**
     * @brief Draws boundary marker labels (-250, +250)
     */
    void drawTicks();
    
    /**
     * @brief Draws current pole angle display
     * 
     * Updates and renders theta text with current angle in degrees,
     * formatted to 2 decimal places.
     */
    void drawTheta();
    
    /**
     * @brief Draws current cart position display
     * 
     * Updates and renders x position text with current cart position,
     * formatted to 2 decimal places.
     */
    void drawX();
};

#endif