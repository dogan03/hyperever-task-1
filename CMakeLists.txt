cmake_minimum_required(VERSION 3.10)
project(CartpoleSimulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(cartpole_sim
    src/main.cpp
    src/CartPole.cpp
    src/MainWindow.cpp
    src/MPC.cpp
    src/CartPoleSimulation.cpp
)

target_include_directories(cartpole_sim PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(cartpole_sim PRIVATE 
    sfml-graphics 
    sfml-window 
    sfml-system
    Eigen3::Eigen
)