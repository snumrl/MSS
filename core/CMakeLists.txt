cmake_minimum_required(VERSION 2.8.6)
project(core)

SET(CMAKE_BUILD_TYPE Release CACHE STRING
	"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel.")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

find_package(DART REQUIRED COMPONENTS collision-bullet CONFIG)
find_package(TinyXML REQUIRED)

include_directories(${DART_INCLUDE_DIRS})

file(GLOB srcs "*.h" "*.cpp")

add_library(core ${srcs})
target_link_libraries(core ${DART_LIBRARIES} ${TinyXML_LIBRARIES})
