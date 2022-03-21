cmake_minimum_required(VERSION 2.8.3)
project(car_demo)

set (PRIUSCUP_MAJOR_VERSION 0)
set (PRIUSCUP_MINOR_VERSION 1)
set (PRIUSCUP_PATCH_VERSION 0)
set (PRIUSCUP_VERSION_FULL ${PRIUSCUP_MAJOR_VERSION}.${PRIUSCUP_MINOR_VERSION}.${PRIUSCUP_PATCH_VERSION})

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  prius_msgs
  cv_bridge
)

find_package(gazebo 11 REQUIRED)
find_package(ignition-msgs5 REQUIRED)
find_package(OpenCV 4.2 REQUIRED imgproc highgui)
find_package(Eigen3 3.3 REQUIRED)

catkin_package(
 # INCLUDE_DIRS include
 # LIBRARIES gazebo_radar_plugin
 CATKIN_DEPENDS gazebo_ros prius_msgs cv_bridge
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc)
target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
add_dependencies(PriusHybridPlugin ${catkin_EXPORTED_TARGETS})


add_executable(image_to_bird_eye_view nodes/image_to_bird_eye_view.cpp)
target_link_libraries(image_to_bird_eye_view
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  ${OpenCV_LIBRARIES})

catkin_install_python(PROGRAMS nodes/joystick_translator
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS PriusHybridPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
