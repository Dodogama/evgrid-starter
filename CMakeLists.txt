cmake_minimum_required(VERSION 3.20)
project(evgrid_starter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)          # -std=c++20, not -std=gnu++20 (portability)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # this is what feeds clangd in VS Code

# A library, not just a pile of .cpp files. Getting used to targets now will
# save you a lot of pain when the work project has 40 of them.
add_library(evgrid
    src/grid.cpp
    src/radar.cpp
)

# PUBLIC = anything linking evgrid also gets this include path.
target_include_directories(evgrid PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Warnings are your C++ tutor. Turn them all on, then don't ignore them.
if(MSVC)
    target_compile_options(evgrid PRIVATE /W4 /permissive-)
else()
    target_compile_options(evgrid PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

add_executable(evgrid_demo src/main.cpp)
target_link_libraries(evgrid_demo PRIVATE evgrid)
