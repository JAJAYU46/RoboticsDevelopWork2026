cmake_minimum_required(VERSION 2.8.12)
project(mo_stereo_camera_driver_own_sample)
option(USING_OPENCV "USING OPENCV" ON)
option(USING_ROS "USING ROS" ON)

set(dep_libs moStereoCameraDriver pthread)


if(USING_ROS)
  # set(ROS_DIR "put your ros install dir here")
  find_package(catkin REQUIRED COMPONENTS
    roscpp
  )
  include_directories(${catkin_INCLUDE_DIRS})
  list(APPEND dep_libs
    ${catkin_LIBRARIES}
  )

    add_definitions(-DUSING_ROS)
endif()

if(USING_OPENCV)
  # set(OpenCV_DIR "put your open_cv install dir here")
  find_package(OpenCV)
  # find_package(OpenCV COMPONENTS
  #      core
  #      highgui
  #      imgproc
  #      imgcodecs
  # )
  include_directories(${OpenCV_INCLUDE_DIRS})
  list(APPEND dep_libs
      ${OpenCV_LIBRARIES}
  )
  add_definitions(-DUSE_OPENCV=1)

endif()
add_definitions(-std=c++11)
include_directories(${CMAKE_SOURCE_DIR}/../include/mo_stereo_camera)
include_directories(${CMAKE_SOURCE_DIR}/include)
link_directories(${CMAKE_SOURCE_DIR}/../lib/)

#if(USING_OPENCV)
#  set(dep_libs
#      moStereoCameraDriver
#      pthread
#      opencv_core
#      opencv_highgui
#      opencv_core
#      opencv_imgproc
#      opencv_imgcodecs)
#else()
#  set(dep_libs moStereoCameraDriver pthread)
#endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

add_executable(get_imu_sample get_imu_sample.cpp common_function.cpp)
target_link_libraries(get_imu_sample ${dep_libs})

