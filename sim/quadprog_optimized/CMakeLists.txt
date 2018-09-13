# This software may be modified and distributed under the terms
# of the MIT license.  See the LICENSE file for details.

file(GLOB sources *.cc)
file(GLOB headers *.hh)
set(library_sources ${sources})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
add_library(quadprog STATIC ${library_sources} ${headers})
set_property(TARGET quadprog PROPERTY POSITION_INDEPENDENT_CODE 1)
set_property(TARGET quadprog PROPERTY PUBLIC_HEADER ${headers})