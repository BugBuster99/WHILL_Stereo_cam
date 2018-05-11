# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(libuvc_camera)
# Load catkin and all dependencies required for this package
find_package(
  catkin REQUIRED COMPONENTS 
  roscpp 
  message_generation
  camera_info_manager 
  dynamic_reconfigure 
  image_transport 
  nodelet 
  sensor_msgs
)

SET(CMAKE_C_FLAGS_DEBUG "-g -DXUNIT_DEBUGGING=1")

generate_dynamic_reconfigure_options(cfg/UVCCamera.cfg)

find_package(libuvc REQUIRED)
message(STATUS "libuvc ${libuvc_VERSION_MAJOR}.${libuvc_VERSION_MINOR}.${libuvc_VERSION_PATCH}")

add_service_files(
  FILES
  SetStreamMode.srv
  GetStreamMode.srv
  GetFirmwareVersion.srv
)

generate_messages(
  DEPENDENCIES
#  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    camera_info_manager
    dynamic_reconfigure
    image_transport
    nodelet
    sensor_msgs
    LIBRARIES libuvc_camera_nodelet
  )

add_definitions(-Dlibuvc_VERSION_MAJOR=${libuvc_VERSION_MAJOR})
add_definitions(-Dlibuvc_VERSION_MINOR=${libuvc_VERSION_MINOR})
add_definitions(-Dlibuvc_VERSION_PATCH=${libuvc_VERSION_PATCH})
include_directories(include ${libuvc_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LINK_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(camera_node src/main.cpp src/camera_driver.cpp)
target_link_libraries(camera_node ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(camera_node ${PROJECT_NAME}_gencfg)

add_library(libuvc_camera_nodelet src/nodelet.cpp src/camera_driver.cpp)
add_dependencies(libuvc_camera_nodelet ${libuvc_camera_EXPORTED_TARGETS})
target_link_libraries(libuvc_camera_nodelet ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(libuvc_camera_nodelet ${PROJECT_NAME}_gencfg)

add_library(libuvc_stereo_nodelet src/stereo_nodelet.cpp src/stereo_driver.cpp src/xunit/xunit_lib_whill.c)
add_dependencies(libuvc_stereo_nodelet ${libuvc_camera_EXPORTED_TARGETS})
target_link_libraries(libuvc_stereo_nodelet ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(libuvc_stereo_nodelet ${PROJECT_NAME}_gencfg)

install(TARGETS camera_node libuvc_camera_nodelet libuvc_stereo_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES libuvc_camera_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
