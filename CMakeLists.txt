cmake_minimum_required(VERSION 2.8)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo REQUIRED)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include(cmake/rospack.cmake)

get_rospack_flags(roscpp)
include_directories(${roscpp_INCLUDE_DIRS})
link_directories(${roscpp_LIBRARY_DIRS})
set(CMAKE_INSTALL_RPATH "${roscpp_LIBRARY_DIRS}")

add_library(my_plugin_with_gazebo SHARED my_plugin_with_gazebo.cc)
set_target_properties(my_plugin_with_gazebo PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")

set_target_properties(my_plugin_with_gazebo PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(my_plugin_with_gazebo ${roscpp_LIBRARIES})
install (TARGETS my_plugin_with_gazebo DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)




