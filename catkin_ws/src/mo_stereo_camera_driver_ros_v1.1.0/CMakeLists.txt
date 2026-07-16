cmake_minimum_required(VERSION 3.0.2)
project(moak_camera)


#option(USING_PCL "USING PCL" OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(MO_TARGET_NAME usb_asic_cam)
set(MO_USB_RAW_TARGET_NAME usb_raw)
set(MO_IMU_TARGET_NAME get_imu_sample)
set(MO_RAW_SAMPLE_TARGET_NAME get_raw_sample)
set(MO_RECTIFY_SAMPLE_TARGET_NAME get_rectify_sample)  # ← 修正：獨立變數

# Extra Executable
set(MO_CAM_RAW_PUBGRAY8_TARGET_NAME cam_raw_publisher_gray8)  
# set(MO_CAM_RECTIFY_PUBGRAY8_TARGET_NAME cam_rectify_publisher_gray8)  


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  camera_info_manager
)
find_package(PCL REQUIRED)
list(FILTER PCL_LIBRARIES EXCLUDE REGEX "vtkpng")
find_package(OpenCV REQUIRED)

# ONNX Runtime（header 直接在 /usr/local/include/ 底下）

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  lib/mo_stereo_camera_driver/include/mo_stereo_camera
  ${CMAKE_CURRENT_SOURCE_DIR}/../mo_stereo_camera_driver_ros_v1.1.0/include
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
    lib/mo_stereo_camera_driver/lib
)

# usb_asic_cam 目標
add_executable(${MO_TARGET_NAME}
    src/usb_asic_cam.cpp
)
target_link_libraries(${MO_TARGET_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    libmoStereoCameraDriver.so
)

# usb_raw 獨立 executable
add_executable(${MO_USB_RAW_TARGET_NAME}
    src/usb_raw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../mo_stereo_camera_driver_ros_v1.1.0/src/common_function.cpp
)
target_include_directories(${MO_USB_RAW_TARGET_NAME} PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    lib/mo_stereo_camera_driver/include/mo_stereo_camera
    ${CMAKE_CURRENT_SOURCE_DIR}/../mo_stereo_camera_driver_ros_v1.1.0/include
)
target_compile_definitions(${MO_USB_RAW_TARGET_NAME} PRIVATE USE_OPENCV)
target_link_libraries(${MO_USB_RAW_TARGET_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    libmoStereoCameraDriver.so
    pthread
    -Wl,--allow-shlib-undefined
    -lz
)

# get_imu_sample 目標
add_executable(${MO_IMU_TARGET_NAME}
    src/get_imu_sample.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../mo_stereo_camera_driver_ros_v1.1.0/src/common_function.cpp
)
target_include_directories(${MO_IMU_TARGET_NAME} PRIVATE
    lib/mo_stereo_camera_driver/include/mo_stereo_camera   # ← 移到最前面，優先搜尋
    ${CMAKE_CURRENT_SOURCE_DIR}/../mo_stereo_camera_driver_ros_v1.1.0/include
    ${catkin_INCLUDE_DIRS}                                  # ← 移到最後面
)
target_link_libraries(${MO_IMU_TARGET_NAME}
    ${catkin_LIBRARIES}
    pthread
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/mo_stereo_camera_driver/lib/libmoStereoCameraDriver.so  # ← 用 CMAKE_CURRENT_SOURCE_DIR 組成完整絕對路徑
)

# get_raw_sample 目標
add_executable(${MO_RAW_SAMPLE_TARGET_NAME}
    src/get_raw_sample.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../mo_stereo_camera_driver_ros_v1.1.0/src/common_function.cpp
)
target_include_directories(${MO_RAW_SAMPLE_TARGET_NAME} PRIVATE
    ${catkin_INCLUDE_DIRS}
    lib/mo_stereo_camera_driver/include/mo_stereo_camera
    ${CMAKE_CURRENT_SOURCE_DIR}/../mo_stereo_camera_driver_ros_v1.1.0/include
)
target_compile_definitions(${MO_RAW_SAMPLE_TARGET_NAME} PRIVATE USE_OPENCV)
target_link_libraries(${MO_RAW_SAMPLE_TARGET_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    libmoStereoCameraDriver.so
    pthread
)

add_executable(raw_publisher
    src/raw_publisher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../mo_stereo_camera_driver_ros_v1.1.0/src/common_function.cpp
)
target_include_directories(raw_publisher PRIVATE
    ${catkin_INCLUDE_DIRS}
    lib/mo_stereo_camera_driver/include/mo_stereo_camera
    ${CMAKE_CURRENT_SOURCE_DIR}/../mo_stereo_camera_driver_ros_v1.1.0/include
)
target_compile_definitions(raw_publisher PRIVATE USE_OPENCV)
target_link_libraries(raw_publisher
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    libmoStereoCameraDriver.so
    pthread
)


add_executable(run
    src/run.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../mo_stereo_camera_driver_ros_v1.1.0/src/common_function.cpp
)
target_include_directories(run PRIVATE
    ${catkin_INCLUDE_DIRS}
    lib/mo_stereo_camera_driver/include/mo_stereo_camera
    ${CMAKE_CURRENT_SOURCE_DIR}/../mo_stereo_camera_driver_ros_v1.1.0/include
)
target_compile_definitions(run PRIVATE USE_OPENCV DEBUG_FRAMENUM_MATCHING TIMING_DIAGNOSTICS)
target_link_libraries(run
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    libmoStereoCameraDriver.so
    pthread
)

# get_rectify_sample 目標  ← 修正：獨立 target，使用獨立變數
add_executable(${MO_RECTIFY_SAMPLE_TARGET_NAME}
    src/get_rectify_sample.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../mo_stereo_camera_driver_ros_v1.1.0/src/common_function.cpp
)
target_include_directories(${MO_RECTIFY_SAMPLE_TARGET_NAME} PRIVATE
    ${catkin_INCLUDE_DIRS}
    lib/mo_stereo_camera_driver/include/mo_stereo_camera
    ${CMAKE_CURRENT_SOURCE_DIR}/../mo_stereo_camera_driver_ros_v1.1.0/include
)
target_compile_definitions(${MO_RECTIFY_SAMPLE_TARGET_NAME} PRIVATE USE_OPENCV)
target_link_libraries(${MO_RECTIFY_SAMPLE_TARGET_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    libmoStereoCameraDriver.so
    pthread
)


# Extra Executable: For CameraCalibration
add_executable(${MO_CAM_RAW_PUBGRAY8_TARGET_NAME} 
    src/camera_calibration/cam_raw_publisher_gray8.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/../mo_stereo_camera_driver_ros_v1.1.0/src/common_function.cpp
)

target_include_directories(${MO_CAM_RAW_PUBGRAY8_TARGET_NAME} PRIVATE
    ${catkin_INCLUDE_DIRS}
    lib/mo_stereo_camera_driver/include/mo_stereo_camera
    ${CMAKE_CURRENT_SOURCE_DIR}/../mo_stereo_camera_driver_ros_v1.1.0/include
)
target_compile_definitions(${MO_CAM_RAW_PUBGRAY8_TARGET_NAME} PRIVATE USE_OPENCV)
target_link_libraries(${MO_CAM_RAW_PUBGRAY8_TARGET_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    libmoStereoCameraDriver.so
    pthread
)



