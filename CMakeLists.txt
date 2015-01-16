cmake_minimum_required(VERSION 2.8.3)
project(depth_calibration)

find_package(catkin REQUIRED COMPONENTS 
    rospy
    roscpp
    std_msgs
    sensor_msgs
    pcl_ros
    pcl_conversions    
    cv_bridge
    image_geometry
)

find_package(Boost REQUIRED thread)

catkin_package(CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    sensor_msgs
    pcl_ros
    pcl_conversions    
    cv_bridge
    image_geometry
)

include_directories(include ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_executable(depth_calibrator src/depth_calibrator.cpp)
target_link_libraries(depth_calibrator ${catkin_LIBRARIES})

add_executable(center_plane_extractor src/center_plane_extractor.cpp)
target_link_libraries(center_plane_extractor ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
    scripts/depth_calibration.py
    scripts/extrinsic_plane_calibration.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
    scripts/depth_calibration.py
    scripts/extrinsic_plane_calibration.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
  launch/depth_calibration.launch
  launch/extrinsic_plane_calibration.launch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)