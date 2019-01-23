cmake_minimum_required(VERSION 2.8.6)
project(gui)

link_directories(../core/)
include_directories(../core/)

find_package(DART REQUIRED COMPONENTS gui CONFIG)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${DART_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

file(GLOB srcs "*.h" "*.cpp")
add_executable(gui ${srcs})
target_link_libraries(gui ${DART_LIBRARIES} GL GLU glut core)
