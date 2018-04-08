cmake_minimum_required(VERSION 2.8.3)
project(cooperative_driving_application)

find_package(catkin REQUIRED COMPONENTS
    roslaunch
    cooperative_driving_common
    cooperative_driving_logic
    cooperative_driving_networking
    dynamic_reconfigure
)

roslaunch_add_file_check(launch/demo_app.xml)

catkin_python_setup()

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  config/dynamic_parameters.cfg
)



catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(catkin COMPONENTS rosunit)
  add_subdirectory(test)
endif (CATKIN_ENABLE_TESTING)
