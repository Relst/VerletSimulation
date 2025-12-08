cmake_minimum_required(VERSION 3.14)

project(VerletSimulation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_compile_options(${OpenMP_CXX_FLAGS})
link_directories("/opt/homebrew/opt/llvm/lib")

find_path(EIGEN3_INCLUDE_DIR Eigen/Dense PATH_SUFFIXES eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(GLEW REQUIRED)

include(FetchContent)

# Fetch GitHub dependency helper
macro(AddModule NAME REPO TAG)
    message(STATUS "Fetching ${NAME} @ ${TAG}")
    FetchContent_Declare(${NAME}
            GIT_REPOSITORY ${REPO}
            GIT_TAG        ${TAG}
            GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(${NAME})
endmacro()

AddModule(IMGUI https://github.com/ocornut/imgui.git v1.91.1)
file(GLOB IMGUI_SRC
        ${imgui_SOURCE_DIR}/*.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
add_library(imgui STATIC ${IMGUI_SRC})
target_link_libraries(imgui PUBLIC glfw)
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)


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
        imgui
)
