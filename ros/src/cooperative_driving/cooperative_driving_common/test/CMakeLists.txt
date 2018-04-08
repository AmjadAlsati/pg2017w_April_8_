set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test_pubscriber_cpp EXCLUDE_FROM_ALL test_pubscriber.cc)
target_link_libraries(test_pubscriber_cpp ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
add_rostest(pubscriber.test DEPENDENCIES test_pubscriber_cpp)
