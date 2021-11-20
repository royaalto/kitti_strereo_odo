### This is a default CMakeLists.txt template that you can use for your project
### It is based on http://wiki.ros.org/catkin/CMakeLists.txt
### You can always remove or comment out stuff you don't need

#######################
## Compiler settings ##
#######################

## Add minimum required cmake version
cmake_minimum_required(VERSION 3.1)
## Project name should match the name of the package you created
project(kitti_stereo_odo)
set(CMAKE_CXX_STANDARD 11)
## CMake flags not used in current stages.
# add_definitions("-Wall")
# add_definitions("-Wextra")
# add_definitions("-Wpedantic")
# add_definitions("-Werror")

## Custom options
# set(MAKE_SHARED False CACHE BOOL "Build and use shared libraries")
# if(NOT MAKE_SHARED)
#     set(CMAKE_FIND_LIBRARY_PREFIXES ".a")
#     set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++ -static")
#     set(BUILD_SHARED_LIBS OFF)
# endif()

##############
## Packages ##
##############

## Find catkin
find_package(catkin REQUIRED)
find_package(G2O REQUIRED)
find_package(OpenCV REQUIRED)
IF(G2O_FOUND)
    include_directories(${G2O_INCLUDE_DIR})
    message("G2O lib is found:"${G2O_INCLUDE_DIR})
ENDIF(G2O_FOUND)
SET(G2O_LIBS
    g2o_cli
    g2o_ext_freeglut_minimal
    g2o_simulator
    g2o_solver_slam2d_linear
    g2o_types_icp
    g2o_types_slam2d
    g2o_core
    g2o_interface
    g2o_solver_csparse
    g2o_solver_structure_only
    g2o_types_sba
    g2o_types_slam3d
    g2o_csparse_extension
    g2o_opengl_helper
    g2o_solver_dense
    g2o_stuff
    g2o_types_sclam2d
    g2o_parser
    g2o_solver_pcg
    g2o_types_data
    g2o_types_sim3
    cxsparse
)
# Find 3rd packages
# find_package(PCL 1.7 REQUIRED)

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    message_filters
)
# Find other packages
find_package(catkin REQUIRED COMPONENTS
    ${${PROJECT_NAME}_CATKIN_DEPS}
)

####################
## Catkin package ##
####################
## Catkin package definition
# INCLUDE_DIRS: which directories should be included
# LIBRARIES: which libraries should be linked
# CATKIN_DEPENDS: This uses the above definition of ${PROJECT}_CATKIN_DEPS
# DEPENDS: OTHER 3RD party libraries that your package requires
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    DEPENDS
)
###########
## Build ##
###########
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Add project source files
set(PROJECT_SOURCE
    src/ros_node.cpp
    src/kitti_stereo_odo.cpp
)
# Make definition of project libs
# if your package needs some library, you can define here
# catkin libraries is needed here
set(PROJECT_LIBS
    pthread
    atomic
    ${catkin_LIBRARIES}
    ${G2O_LIBS}
    fmt
)

# If you are making some shared libraries this will word
# if(MAKE_SHARED)
#     add_library(${PROJECT_NAME} ${PROJECT_SOURCE})
# else()
#     add_library(${PROJECT_NAME} STATIC ${PROJECT_SOURCE})
# endif()

# Executables and target dependencies
add_executable(kitti_stereo_odo
    src/kitti_stereo_odo.cpp
    src/ros_node.cpp
    ${PROJECT_SOURCE}
)
# Avoid compile failure
add_dependencies(kitti_stereo_odo ${kitti_stereo_odo_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kitti_stereo_odo ${PRJECT_NAME} ${PROJECT_LIBS})

#############
## Install ##
#############

install(
    TARGETS
    kitti_stereo_odo
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
    config
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
