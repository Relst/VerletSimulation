cmake_minimum_required(VERSION 3.14)

# Use Homebrew LLVM clang/clang++ with OpenMP support
set(CMAKE_C_COMPILER "/opt/homebrew/opt/llvm/bin/clang")
set(CMAKE_CXX_COMPILER "/opt/homebrew/opt/llvm/bin/clang++")

project(VerletSimulation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenMP manual flags
set(OpenMP_C_FLAGS "-fopenmp")
set(OpenMP_C_LIB_NAMES "omp")
set(OpenMP_CXX_FLAGS "-fopenmp")
set(OpenMP_CXX_LIB_NAMES "omp")
set(OpenMP_omp_LIBRARY "/opt/homebrew/opt/llvm/lib/libomp.dylib")

add_compile_options(${OpenMP_CXX_FLAGS})
link_directories("/opt/homebrew/opt/llvm/lib")

find_path(EIGEN3_INCLUDE_DIR Eigen/Dense PATH_SUFFIXES eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(GLEW REQUIRED)

add_executable(VerletSimulation
        main.cpp
        Balls/Ball.cpp
        Balls/Ball.h
        Line.cpp
)

target_include_directories(VerletSimulation PRIVATE
        /opt/homebrew/opt/llvm/include
        ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(VerletSimulation PRIVATE
        GLEW::GLEW
        OpenGL::GL
        glfw
        /opt/homebrew/opt/llvm/lib/libomp.dylib
)
