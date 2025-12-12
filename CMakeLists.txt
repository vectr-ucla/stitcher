cmake_minimum_required(VERSION 3.10)

# set the project name
project(stitcher_planner)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

#including eigen dir
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)
include_directories(${YAMLCPP_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS 
	  std_msgs
    nav_msgs 
    sensor_msgs
  LIBRARIES trajGenerator
)

include_directories(
   ${PROJECT_SOURCE_DIR}
   ${catkin_INCLUDE_DIRS}
)

add_library(trajGenerator src/primitives/lqmt/LQMTPrimitive.cpp
                          src/primitives/min_time/MinTimePrimitive.cpp
                          src/collision_checker/CollisionChecker.cc src/collision_checker/CollisionChecker.tpp
                          src/feasibility_checker/FeasibilityChecker.cpp src/feasibility_checker/FeasibilityChecker.tpp
                          src/geometric_planner/astar/Astar.cpp src/geometric_planner/jps/Jps3d.cpp src/geometric_planner/GeometricPlanner.cpp
                          src/reference_path_utils/ReferencePath.cpp 
                          src/planners/heuristic_generator/HeuristicDP.cpp
                          src/planners/stitcher/STITCHER.tpp
                          src/planners/stitcher/LqmtSTITCHER.cpp)
            
target_include_directories(trajGenerator PUBLIC src src/primitives/lqmt 
                                                src/primitives/min_time 
                                                src/collision_checker 
                                                src/geometric_planner/astar 
                                                src/geometric_planner/jps
                                                src/geometric_planner 
                                                src/reference_path_utils 
                                                src/feasibility_checker
                                                src/planners/heuristic_generator
                                                src/planners/stitcher)

################# EXECUTABLES FOR PATH PLOTTING NODES ##########################
# planning with goal click for start and goal
add_executable(goal_click_node  src/node/ROS_Goal_Click_Planning_Main.cc src/node/ROS_Goal_Click_Planning.cc)
add_dependencies(goal_click_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goal_click_node trajGenerator ${catkin_LIBRARIES})
############################
