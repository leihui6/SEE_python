cmake_minimum_required(VERSION 3.10)
project(pysee VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_GENERATOR_PLATFORM x64)

# Add source files
set(SOURCES
    test_main.cpp
    see_core/src/see_core.cpp
    SEE.cpp
    python_bindings.cpp
)

# Add header files
set(HEADERS
    see_core/inc/see_core.h
    see_core/inc/see_params.h
    see_core/inc/see_structs.h
    see_core/inc/view_opt.h
    see_common/inc/abstract_core.h
    see_common/inc/common_params.h
    see_common/inc/common_structs.h
    see_common/inc/pcl_common.h
    see_common/inc/pcl_types.h
    SEE.h
)

# Find required packages
find_package(PCL REQUIRED)
message(STATUS "Lemon include directories: ${CMAKE_CURRENT_SOURCE_DIR}/3rdPatry/lemon/include")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdPatry/lemon/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdPatry/pybind11/include)  
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdPatry)  
include_directories($ENV{QHULL})  

include_directories($ENV{NLOPT}/include)  
# link_directories($ENV{NLOPT}/lib)

include_directories($ENV{PYTHON}/include)  
link_directories($ENV{PYTHON}/libs)


# Specify include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/see_core/inc
    ${PROJECT_SOURCE_DIR}/see_common/inc
    ${PROJECT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)

# Add definitions and link directories for PCL
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})


# Create the pybind11 module
add_library(pysee MODULE ${SOURCES} ${HEADERS})
target_link_libraries(pysee PRIVATE ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} $ENV{NLOPT}/lib/nlopt.lib)

# Adjust the output name of the module
set_target_properties(pysee PROPERTIES PREFIX "")
if(WIN32)
    set_target_properties(pysee PROPERTIES SUFFIX ".pyd")
else()
    set_target_properties(pysee PROPERTIES SUFFIX ".so")
endif()

# Add compiler options
if(MSVC)
    target_compile_options(pysee PRIVATE /W4)
else()
    target_compile_options(pysee PRIVATE -Wall -Wextra -pedantic)
endif()