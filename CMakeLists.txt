cmake_minimum_required(VERSION 2.8.3)
project(costmap_2d)
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O2  ")
find_package(
  Boost REQUIRED COMPONENTS system
        )
find_package(OpenCV REQUIRED)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
include_directories(
    include
)

include_directories(${Boost_INCLUDE_DIRS}
                     ${OpenCV_INCLUDE_DIRS})


add_library(costmap_2d_lib
  src/costmap_2d/array_parser.cpp
  src/costmap_2d/costmap_2d.cpp
  src/costmap_2d/layer.cpp
  src/costmap_2d/layered_costmap.cpp
  src/costmap_2d/costmap_2d_ros.cpp
  src/costmap_2d/costmap_math.cpp
  src/costmap_2d/footprint.cpp
  src/costmap_2d/costmap_layer.cpp
)

add_library(a_star_lib
  src/global_planner/a_star.cpp
  src/global_planner/astar.cpp
  src/global_planner/grid_path.cpp
  src/global_planner/gradient_path.cpp
  src/global_planner/quadratic_calculator.cpp
)

add_library(layers
  plugins/inflation_layer.cpp
  plugins/obstacle_layer.cpp
  plugins/static_layer.cpp
)


target_link_libraries(layers costmap_2d_lib )

add_executable(costmap main.cpp)

target_link_libraries(
        costmap
        costmap_2d_lib
        layers
        a_star_lib
 	${Boost_LIBRARIES}
        pthread 
        boost_thread
        ${OpenCV_LIBS}
)
