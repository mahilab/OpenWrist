cmake_minimum_required(VERSION 3.7)

set(MEL_ROOT "C:/Git/MEL" CACHE FILEPATH "Absolute path to MEL.")
set(QUANSER_ROOT "C:/Program Files/Quanser" CACHE FILEPATH "Absolute path to Quanser installation.")
set(EIGEN_ROOT   "C:/dev/eigen"             CACHE FILEPATH "Absolute path to Eigen installation.")
set(SFML_ROOT    "C:/Git/SFML"              CACHE FILEPATH "Absolute path to SFML")

# enable C++11
set(CMAKE_CXX_STANDARD 11)

# create project
project(OpenWrist)

# add configurations
set(CMAKE_CONFIGURATION_TYPES Release)

# set binary output location (optional, but recommended)
if(WIN32)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/windows)
else()
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/linux)
endif()

# add definitons
if(WIN32)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS -DNOMINMAX -D_WINSOCK_DEPRECATED_NO_WARNINGS)
endif()

# include directories (your includes and MEL's)
include_directories(
    "src"
    "../MEII/include"
    "../MEL/include"
    "C:/dev/myo-sdk-win-0.9.0/include"
    "${SFML_ROOT}/include"
    ${EIGEN_ROOT}
)

# link MEL and other libs
set(LINK_LIBS "")
if(WIN32)
    link_directories("${MEL_ROOT}/lib/windows/Release")
    if (CMAKE_SIZEOF_VOID_P EQUAL 8) # 64-bit
        link_directories("${QUANSER_ROOT}/QUARC/lib/win64")
    else()
        link_directories("${QUANSER_ROOT}/QUARC/lib/windows")
    endif()
    link_directories("C:/dev/myo-sdk-win-0.9.0/lib")
    link_directories("${SFML_ROOT}/lib")
    link_directories("../MEII/lib/Release")
    list(APPEND LINK_LIBS
        MEL
        ws2_32
        winmm
        Pdh
        Psapi
        hil
        quanser_communications
        quanser_runtime
        quanser_common
        bufferoverflowU
        legacy_stdio_definitions
        XInput)
endif()

# get common files

set(OPENWRIST_SRC
    src/OpenWrist.hpp
    src/OwConfiguration
    src/OwParameters
    src/OpenWrist.cpp
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
    src/Games/Jedi.cpp
    src/Games/Jedi.hpp
    src/Games/OctagonSqueeze/OctagonSqueeze.hpp
    src/Games/OctagonSqueeze/OctagonSqueeze.cpp
    src/HapticGuidance/Pendulum.cpp
    src/HapticGuidance/Pendulum.hpp
    src/HapticTraining/BallAndBeam.cpp
    src/HapticTraining/BallAndBeam.hpp)
target_link_libraries(demos ${LINK_LIBS})

# Haptics Symposium 2018 Experiments
add_executable(haptic_guidance ${OPENWRIST_SRC} ${CUFF_SRC}
    src/HapticGuidance/main.cpp
    src/HapticGuidance/HapticGuidance.hpp
    src/HapticGuidance/HapticGuidance.cpp
    src/HapticGuidance/Pendulum.hpp
    src/HapticGuidance/Pendulum.cpp)
target_link_libraries(haptic_guidance ${LINK_LIBS})

# ToH 2019 Experiments
add_executable(haptic_training ${OPENWRIST_SRC} ${CUFF_SRC}
    src/HapticTraining/main.cpp
    src/HapticTraining/FurutaPendulum.hpp
    src/HapticTraining/FurutaPendulum.cpp
    src/HapticTraining/BallAndBeam.hpp
    src/HapticTraining/BallAndBeam.cpp
    src/HapticTraining/HapticTraining.hpp
    src/HapticTraining/HapticTraining.cpp)
target_link_libraries(haptic_training ${LINK_LIBS})

# Myo EMG Control - ELEC 575 2018 Final Project
add_executable(myo ${OPENWRIST_SRC} src/Myo/main.cpp src/Myo/MyoBand.cpp src/Myo/MyoBand.hpp)
target_link_libraries(myo ${LINK_LIBS} myo64 MEII)

add_executable(xwing ${OPENWRIST_SRC}
    src/Games/XWing/main.cpp
    src/Games/XWing/XWing.hpp
    src/Games/XWing/XWing.cpp)
target_link_libraries(xwing ${LINK_LIBS})

add_executable(testing tests/testing.cpp ${OPENWRIST_SRC})
target_link_libraries(testing ${LINK_LIBS})

add_executable(dylan src/Dylan/main.cpp ${OPENWRIST_SRC})
target_link_libraries(dylan ${LINK_LIBS})
