
# include_directories(/home/dygg/armlib/eigen339/include/eigen3)

add_library(Imu_Integrator_node src/Imu_Integrator.cpp)
# target_link_libraries(Imu_Integrator_node ${catkin_LIBRARIES})
target_include_directories(Imu_Integrator_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ../third_lib/eigen3)
# target_include_directories(Imu_Integrator_node PUBLIC )

#using EIGEN
# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})

