cmake_minimum_required(VERSION 3.7)

set(EIGEN_ROOT "C:/dev/eigen" CACHE FILEPATH "Absolute path to Eigen installation.")

# create project
project(OpenWrist VERSION 0.1.0 LANGUAGES CXX)

# set binary output location (optional, but recommended)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# include directories (your includes and MEL's)
include_directories("src" ${EIGEN_ROOT})

# find MEL
find_package(MEL REQUIRED)

# get common files
set(OPENWRIST_SRC
    src/OpenWrist.hpp
    src/OpenWrist.cpp
    src/OwConfiguration.hpp
    src/OwConfiguration.cpp
    src/OpenWristSim.hpp
    src/OpenWristSim.cpp
    src/OwParameters.hpp
    src/OwConfiguration.cpp)

set(CUFF_SRC
    src/Cuff/Cuff.hpp
    src/Cuff/Cuff.cpp
    src/Cuff/definitions.h
    src/Cuff/commands.h
    src/Cuff/qbmove_communications.h
    src/Cuff/qbmove_communications.cpp)

#==============================================================================
# APPLICATIONS
#==============================================================================

# Various Demos and Utilities
add_executable(demos ${OPENWRIST_SRC}
    src/demos.cpp
    src/Games/Jedi/Jedi.cpp
    src/Games/Jedi/Jedi.hpp
    src/Games/OctagonSqueeze/OctagonSqueeze.hpp
    src/Games/OctagonSqueeze/OctagonSqueeze.cpp
    src/Games/Airplane/Airplane.cpp
    src/Games/Airplane/Airplane.hpp
    src/Simulations/Pendulum.cpp
    src/Simulations/Pendulum.hpp
    src/Simulations/BallAndBeam.cpp
    src/Simulations/BallAndBeam.hpp
    ${CUFF_SRC}
)
target_link_libraries(demos MEL::quanser)

# ToH 2019 Experiments
add_executable(haptic_training ${OPENWRIST_SRC} ${CUFF_SRC}
    src/HapticTraining/main.cpp
    src/Simulations/FurutaPendulum.hpp
    src/Simulations/FurutaPendulum.cpp
    src/Simulations/BallAndBeam.hpp
    src/Simulations/BallAndBeam.cpp
    src/HapticTraining/HapticTraining.hpp
    src/HapticTraining/HapticTraining.cpp
    src/HapticTraining/main2.cpp)
target_link_libraries(haptic_training MEL::quanser)

# X-wing
add_executable(xwing ${OPENWRIST_SRC}
    src/Games/XWing/main.cpp
    src/Games/XWing/XWing.hpp
    src/Games/XWing/XWing.cpp)
target_link_libraries(xwing MEL::quanser)

# Caz tests
add_executable(testing tests/testing.cpp ${OPENWRIST_SRC})
target_link_libraries(testing MEL::quanser)

# GUI Demo Launcher
find_package(carnot)
if (carnot_FOUND)
    add_executable(demo-gui src/GUI/demo.cpp ${OPENWRIST_SRC})
    target_link_libraries(demo-gui MEL::quanser carnot)
endif()