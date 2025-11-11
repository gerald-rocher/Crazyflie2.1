cmake_minimum_required(VERSION 3.16)
project(wind_publisher_system_plugin)

find_package(PkgConfig REQUIRED)

pkg_check_modules(GZ_SIM gz-sim8)
pkg_check_modules(GZ_TRANSPORT gz-transport13)
pkg_check_modules(GZ_MSGS gz-msgs10)
pkg_check_modules(GZ_MATH gz-math7)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(gz-sim8-wind-publisher-system SHARED 
  src/wind_publisher_system.cc
)

target_include_directories(gz-sim8-wind-publisher-system PRIVATE
  ${GZ_SIM_INCLUDE_DIRS}
  ${GZ_TRANSPORT_INCLUDE_DIRS}
  ${GZ_MSGS_INCLUDE_DIRS}
  ${GZ_MATH_INCLUDE_DIRS}
)

target_link_libraries(gz-sim8-wind-publisher-system
  ${GZ_SIM_LIBRARIES}
  ${GZ_TRANSPORT_LIBRARIES}
  ${GZ_MSGS_LIBRARIES}
  ${GZ_MATH_LIBRARIES}
)

install(TARGETS gz-sim8-wind-publisher-system
  DESTINATION lib
)