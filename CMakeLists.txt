cmake_minimum_required(VERSION 2.8.3)
project(fast_lio)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# find_package(PythonLibs REQUIRED)
# find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)

add_message_files(
  FILES
  Pose6D.msg
  CustomMsg.msg
  CustomPoint.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 # PCL
  INCLUDE_DIRS
)

add_executable(fastlio_mapping src/aggregate_pcds.cpp src/laserMapping.cpp src/lidar_localizer.cpp src/IMU_Processing.cpp src/pcl_utils.cpp
    include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp src/dist_checkup.cpp)
target_link_libraries(fastlio_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(check_traj_in_tls src/check_traj_in_tls.cpp src/dist_checkup.cpp)
target_link_libraries(check_traj_in_tls ${PCL_LIBRARIES})

add_executable(rectify_hovermap_pcl src/rectify_hovermap_pcl.cpp)
target_link_libraries(rectify_hovermap_pcl ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(aggregate_pcds src/aggregate_pcds_app.cpp src/aggregate_pcds.cpp)
target_link_libraries(aggregate_pcds ${PCL_LIBRARIES})
