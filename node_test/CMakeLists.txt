cmake_minimum_required(VERSION 2.8.3)
project(node_test)

find_package(catkin REQUIRED COMPONENTS
    rospy
    rospy_message_converter
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS
        rospy
        rospy_message_converter
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
   nodes/test_service
   nodes/test_filter
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(roslint)
  roslint_python()
  roslint_add_test()
  #catkin_add_nosetests(tests)
endif()