cmake_minimum_required(VERSION 3.10)

project(pysee VERSION 1.0 LANGUAGES C CXX)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Find Python package
find_package(Python3 3.8 EXACT REQUIRED COMPONENTS Interpreter Development)

# Print Python information
message(STATUS "Python3_FOUND: ${Python3_FOUND}")
message(STATUS "Python3_EXECUTABLE: ${Python3_EXECUTABLE}")
message(STATUS "Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}")
message(STATUS "Python3_LIBRARIES: ${Python3_LIBRARIES}")

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

# Set include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdPatry/lemon/include
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdPatry/pybind11/include
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdPatry
    $ENV{QHULL}
    $ENV{NLOPT}/include
    ${Python3_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/see_core/inc
    ${PROJECT_SOURCE_DIR}/see_common/inc
    ${PROJECT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenMP_CXX_INCLUDE_DIRS}
)

# Add definitions and link directories for PCL
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# Set RPATH for the built library
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Create the pybind11 module
add_library(pysee MODULE ${SOURCES} ${HEADERS})

# Set NLOPT library name based on the operating system
if(WIN32)
    set(NLOPT_LIBRARY "$ENV{NLOPT}/lib/nlopt.lib")
else()
    set(NLOPT_LIBRARY "$ENV{NLOPT}/lib/libnlopt.so")
endif()

# Link libraries
target_link_libraries(pysee PRIVATE 
    ${PCL_LIBRARIES} 
    ${Python3_LIBRARIES}
    ${NLOPT_LIBRARY}
    OpenMP::OpenMP_CXX
)

# Explicitly add OpenMP compile options and link flags
target_compile_options(pysee PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(pysee PRIVATE ${OpenMP_CXX_FLAGS})

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

# Print debug information
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "CXX Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "NLOPT_LIBRARY: ${NLOPT_LIBRARY}")
message(STATUS "OpenMP_CXX_FOUND: ${OpenMP_CXX_FOUND}")
message(STATUS "OpenMP_CXX_FLAGS: ${OpenMP_CXX_FLAGS}")
message(STATUS "OpenMP_CXX_INCLUDE_DIRS: ${OpenMP_CXX_INCLUDE_DIRS}")