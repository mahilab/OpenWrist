cmake_minimum_required(VERSION 3.7)

set(MEL_ROOT "C:/Git/MEL" CACHE FILEPATH "Absolute path to MEL.")
set(QUANSER_ROOT "C:/Program Files/Quanser" CACHE FILEPATH "Absolute path to Quanser installation.")

# enable C++11
set(CMAKE_CXX_STANDARD 11)

# create project
project(OpenWrist)

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
include_directories("src" "../MEL/include")

# link MEL and other libs
set(LINK_LIBS "")
if(WIN32)
    link_directories("${MEL_ROOT}/lib/windows/Release")
    if (CMAKE_SIZEOF_VOID_P EQUAL 8) # 64-bit
        link_directories("${QUANSER_ROOT}/QUARC/lib/win64")
    else()
        link_directories("${QUANSER_ROOT}/QUARC/lib/windows")
    endif()
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
        legacy_stdio_definitions)
endif()

# get common files
set(COMMON src/OpenWrist.hpp src/OwConfiguration src/OwParameters src/OpenWrist.cpp src/OwConfiguration.cpp)

# create application(s)
add_executable(demos ${COMMON}
    src/demos.cpp
    src/Games/Jedi.cpp
    src/Games/Jedi.hpp
    src/HapticGuidance/Pendulum.cpp
    src/HapticGuidance/Pendulum.hpp
    src/HapticGuidance/BallAndBeam.cpp
    src/HapticGuidance/BallAndBeam.hpp)
target_link_libraries(demos ${LINK_LIBS})

add_executable(haptic_guidance ${COMMON}
    src/HapticGuidance/main.cpp
    src/HapticGuidance/FurutaPendulum.hpp
    src/HapticGuidance/FurutaPendulum.cpp
    src/HapticGuidance/Pendulum.hpp
    src/HapticGuidance/Pendulum.cpp
    src/HapticGuidance/BallAndBeam.hpp
    src/HapticGuidance/BallAndBeam.cpp
    src/Cuff/Cuff.hpp
    src/Cuff/Cuff.cpp
    src/Cuff/definitions.h
    src/Cuff/commands.h
    src/Cuff/qbmove_communications.h
    src/Cuff/qbmove_communications.cpp)
target_link_libraries(haptic_guidance ${LINK_LIBS})
