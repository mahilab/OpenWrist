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
include_directories("include" "../MEL/include")

# link MEL and other libs
set(LINK_LIBS "")
if(WIN32)
    link_directories("${MEL_ROOT}/lib/windows/Debug")
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

# create application(s)
add_executable(demos src/demos.cpp src/Jedi.cpp include/Jedi.hpp src/Pendulum.cpp include/Pendulum.hpp)
target_link_libraries(demos ${LINK_LIBS})

