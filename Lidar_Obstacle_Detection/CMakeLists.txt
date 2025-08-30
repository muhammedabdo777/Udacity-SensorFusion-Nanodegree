cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
   
   add_definitions(-std=c++14)
   
   set(CXX_FLAGS "-Wall")
   set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
   
   project(LidarProject)
   
   find_package(PCL 1.2 REQUIRED)
   find_package(Boost REQUIRED COMPONENTS filesystem system)

include_directories(
    ${PCL_INCLUDE_DIRS} 
    ${Boost_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)
link_directories(
    ${PCL_LIBRARY_DIRS} 
    ${Boost_LIBRARY_DIRS}
)
   add_definitions(${PCL_DEFINITIONS})
   list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
   
   
   add_executable (environment src/environment.cpp src/processPointClouds.cpp src/render.cpp src/ransac.cpp)
   target_link_libraries (environment ${PCL_LIBRARIES} ${Boost_LIBRARIES}  Boost::filesystem Boost::system)




