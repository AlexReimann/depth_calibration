cmake_minimum_required(VERSION 2.8.3)
project(depth_calibration)

find_package(catkin REQUIRED
    rospy
    roscpp
    std_msgs
    sensor_msgs
    pcl_ros
    pcl_conversions    
    cv_bridge
    image_geometry
    nodelet
)

find_package(Boost REQUIRED COMPONENTS thread filesystem)
find_package(OpenCV REQUIRED)

catkin_package(
  DEPENDS
    OpenCV
  CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    sensor_msgs
    pcl_ros
    pcl_conversions    
    cv_bridge
    image_geometry
)

include_directories(include ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    src/depth_adjuster.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(depth_calibrator src/depth_calibrator.cpp)
target_link_libraries(depth_calibrator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(center_plane_extractor src/center_plane_extractor.cpp)
target_link_libraries(center_plane_extractor ${catkin_LIBRARIES})

##############################################################################
# Installs
##############################################################################

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
    scripts/depth_calibration.py
    scripts/extrinsic_plane_calibration.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
    scripts/depth_calibration.py
    scripts/extrinsic_plane_calibration.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
