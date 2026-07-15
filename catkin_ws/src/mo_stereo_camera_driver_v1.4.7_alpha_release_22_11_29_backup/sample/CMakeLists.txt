cmake_minimum_required(VERSION 2.8.12)
project(mo_stereo_camera_driver_sample)
option(USING_OPENCV "USING OPENCV" ON)
if(USING_OPENCV)
  # set(OpenCV_DIR "put your open_cv install dir here")
  find_package(OpenCV)
  include_directories(${OpenCV_INCLUDE_DIRS})
  add_definitions(-DUSE_OPENCV=1)

endif()
add_definitions(-std=c++11)
include_directories(${CMAKE_SOURCE_DIR}/../include/mo_stereo_camera)
include_directories(${CMAKE_SOURCE_DIR}/include)
link_directories(${CMAKE_SOURCE_DIR}/../lib/)

if(USING_OPENCV)
  set(dep_libs
      moStereoCameraDriver
      pthread
      opencv_core
      opencv_highgui
      opencv_core
      opencv_imgproc
      opencv_imgcodecs)
else()
  set(dep_libs moStereoCameraDriver pthread)
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

add_executable(get_raw_sample get_raw_sample.cpp common_function.cpp)
target_link_libraries(get_raw_sample ${dep_libs})

add_executable(get_rectify_sample get_rectify_sample.cpp common_function.cpp)
target_link_libraries(get_rectify_sample ${dep_libs})

add_executable(get_rgbd_sample get_rgbd_sample.cpp common_function.cpp)
target_link_libraries(get_rgbd_sample ${dep_libs})

add_executable(get_dense_rgbd_sample get_dense_rgbd_sample.cpp
                                     common_function.cpp)
target_link_libraries(get_dense_rgbd_sample ${dep_libs})

add_executable(get_imu_sample get_imu_sample.cpp common_function.cpp)
target_link_libraries(get_imu_sample ${dep_libs})

add_executable(set_camera_sample set_camera_sample.cpp common_function.cpp)
target_link_libraries(set_camera_sample ${dep_libs})

add_executable(get_point_cloud_sample get_point_cloud_sample.cpp
                                      common_function.cpp)
target_link_libraries(get_point_cloud_sample ${dep_libs})

add_executable(get_multi_camera_sample get_multi_camera_sample.cpp)
target_link_libraries(get_multi_camera_sample ${dep_libs})
