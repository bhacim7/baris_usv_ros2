cmake_minimum_required(VERSION 3.16)
project(usv_gz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

# Pressure integration: header only, needs gz-math alone, so it is testable without Gazebo
add_library(hull_hydrostatics INTERFACE)
target_include_directories(hull_hydrostatics INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(hull_hydrostatics INTERFACE gz-math7::gz-math7)

# The wave surface comes from VRX's Wavefield class, so the boat feels exactly the waves
# that are drawn. VRX does not install that header, so it is taken from the VRX sources
# baked into the image. Without VRX (e.g. on the boat computer) the system is skipped.
set(VRX_WS "/opt/vrx_ws" CACHE PATH "VRX workspace (sources in src/, build in install/)")
find_path(VRX_WAVEFIELD_INCLUDE_DIR Wavefield.hh
  PATHS "${VRX_WS}/src/vrx/vrx_gz/src" NO_DEFAULT_PATH)
find_library(VRX_WAVES_LIBRARY Waves
  PATHS "${VRX_WS}/install/lib" NO_DEFAULT_PATH)

if(VRX_WAVEFIELD_INCLUDE_DIR AND VRX_WAVES_LIBRARY)
  add_library(UsvHullBuoyancy SHARED src/HullBuoyancy.cc)
  target_include_directories(UsvHullBuoyancy PRIVATE "${VRX_WAVEFIELD_INCLUDE_DIR}")
  target_link_libraries(UsvHullBuoyancy PRIVATE
    hull_hydrostatics
    gz-sim8::core
    gz-plugin2::gz-plugin2
    "${VRX_WAVES_LIBRARY}")
  # libWaves.so lives outside the usual library paths
  get_filename_component(VRX_WAVES_DIR "${VRX_WAVES_LIBRARY}" DIRECTORY)
  set_target_properties(UsvHullBuoyancy PROPERTIES INSTALL_RPATH "${VRX_WAVES_DIR}")
  install(TARGETS UsvHullBuoyancy DESTINATION lib)
else()
  message(WARNING "VRX Wavefield not found under ${VRX_WS}: HullBuoyancy is not built")
endif()

# Lets `gz sim` find the system after `source install/setup.bash`
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_hull_hydrostatics test/test_hull_hydrostatics.cpp)
    target_link_libraries(test_hull_hydrostatics hull_hydrostatics)
  else()
    message(WARNING "ament_cmake_gtest not found: hydrostatics tests are not built")
  endif()
endif()

ament_package()
