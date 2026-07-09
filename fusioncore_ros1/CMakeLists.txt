cmake_minimum_required(VERSION 3.0.2)
project(fusioncore_ros1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  geographic_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(PROJ QUIET)
if(NOT PROJ_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(PROJ REQUIRED proj)
  if(NOT TARGET PROJ::proj)
    add_library(PROJ::proj INTERFACE IMPORTED)
    target_include_directories(PROJ::proj INTERFACE ${PROJ_INCLUDE_DIRS})
    target_link_libraries(PROJ::proj INTERFACE ${PROJ_LINK_LIBRARIES})
  endif()
endif()

add_message_files(
  FILES
    Azimuth.msg
    GnssStatus.msg
    FilterHealth.msg
)

add_service_files(
  FILES
    FromLL.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geographic_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    geographic_msgs
    diagnostic_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_srvs
    message_runtime
)

set(FUSIONCORE_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../fusioncore_core)

add_executable(fusioncore_node
  src/fusion_node.cpp
  ${FUSIONCORE_CORE_DIR}/src/ukf.cpp
  ${FUSIONCORE_CORE_DIR}/src/fusioncore.cpp
  ${FUSIONCORE_CORE_DIR}/src/motion_model.cpp
)

add_dependencies(fusioncore_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(fusioncore_node PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${FUSIONCORE_CORE_DIR}/include
)

target_link_libraries(fusioncore_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  PROJ::proj
)

install(TARGETS fusioncore_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_proj_axis_order tests/test_proj_axis_order.cpp)
  if(TARGET test_proj_axis_order)
    target_link_libraries(test_proj_axis_order PROJ::proj)
  endif()
endif()
