cmake_minimum_required(VERSION 3.10)
project(armor_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)
add_definitions(-O3)


## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Include vcpkg setup
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/vcpkg_setup.cmake OPTIONAL)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb)
find_package(cv_bridge REQUIRED)
# Use vcpkg's modern cmake config for g2o
find_package(g2o CONFIG REQUIRED)
find_package(fmt REQUIRED)
find_package(Sophus CONFIG REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC
${OpenCV_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
g2o::core
g2o::stuff
g2o::solver_csparse
g2o::types_sba
g2o::types_slam3d
g2o::solver_dense
fmt::fmt
Sophus::Sophus
tbb
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN aurora::auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_lint_cmake
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_detector test/test_detector.cpp)
  target_link_libraries(test_detector ${PROJECT_NAME})

endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  docs
  model
)
