cmake_minimum_required(VERSION 3.10)
project(SUPER_SLAM)

# --- 1. Settings ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

# --- 2. Dependencies ---
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

# --- 3. Include Directories ---
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
)

# --- 4. Define Source Files ---
# Core sources (explicit files)
set(SLAM_CORE_SRC
    src/camera.cpp
    src/frame.cpp
    src/pnp_solver.cpp
)
# Other sources
file(GLOB IO_MODULE_SRC modules/io/*.cpp)
file(GLOB FRONTEND_MODULE_SRC modules/frontend/*.cpp)
file(GLOB BACKEND_MODULE_SRC modules/backend/*.cpp)
file(GLOB GEOMETRY_MODULE_SRC modules/geometry/*.cpp)
file(GLOB VIZ_MODULE_SRC modules/viz/*.cpp)

# Combine all sources
set(SLAM_SOURCES
    ${SLAM_CORE_SRC}
    ${IO_MODULE_SRC}
    ${FRONTEND_MODULE_SRC}
    ${BACKEND_MODULE_SRC}
    ${GEOMETRY_MODULE_SRC}
    ${VIZ_MODULE_SRC}
)

# --- 5. Create Library ---
add_library(super_slam_lib ${SLAM_SOURCES})

target_link_libraries(super_slam_lib
    ${OpenCV_LIBS}
    yaml-cpp
    Python3::Python
)

# --- 6. Create Executable ---
add_executable(super_slam src/main.cpp)

target_link_libraries(super_slam
    super_slam_lib
    ${OpenCV_LIBS}
    Python3::Python
)

# --- 7. Messages ---
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "Eigen3 Include Dir: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "yaml-cpp Include Dir: ${yaml-cpp_INCLUDE_DIRS}")
message(STATUS "Python Include Dirs: ${Python3_INCLUDE_DIRS}")
message(STATUS "NumPy Include Dirs: ${Python3_NumPy_INCLUDE_DIRS}")
message(STATUS "Source Files: ${SLAM_SOURCES}")
message(STATUS "Build Complete!")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
