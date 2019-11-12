cmake_minimum_required(VERSION 2.8)
project(region_detection)

add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  octomap_ros
  octomap_msgs
)

find_package(octomap REQUIRED)
include_directories(SYSTEM BEFORE ${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ndt_omp octomap
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS}
)

add_library(ndt_omp
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)

add_executable(align
  apps/align.cpp
)

add_dependencies(align
  ndt_omp
)
target_link_libraries(align
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
)

add_executable(showNDTVoxelwithThreeType apps/showNDTVoxelwithThreeType.cpp)
add_dependencies(showNDTVoxelwithThreeType  ndt_omp)
target_link_libraries(showNDTVoxelwithThreeType  ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  ndt_omp)

add_executable(region_detection apps/region_detection.cpp)
add_dependencies(region_detection  ndt_omp)
target_link_libraries(region_detection  ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  ndt_omp)

add_executable(autoSelectPlaneWithNdt apps/autoSelectPlaneWithNdt.cpp)
add_dependencies(autoSelectPlaneWithNdt  ndt_omp)
target_link_libraries(autoSelectPlaneWithNdt  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ndt_omp )

add_executable(switchPointCloud apps/switchPointCloud.cpp)
add_dependencies(switchPointCloud  ndt_omp)
target_link_libraries(switchPointCloud  ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  ndt_omp)
