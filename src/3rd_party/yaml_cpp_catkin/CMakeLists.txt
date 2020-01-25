cmake_minimum_required(VERSION 2.8.3)
project(yaml_cpp_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

# Release versions currently don't use C++11 and create more compiler warnings.
set(YAML_CPP_TAG 11607eb5bf1258641d80f7051e7cf09e317b4746)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(yaml_cpp_src
  DOWNLOAD_COMMAND cp ${CMAKE_SOURCE_DIR}/yaml_cpp_src.tar.gz yaml_cpp_src.tar.gz 
  #GIT_REPOSITORY  https://github.com/jbeder/yaml-cpp
  #GIT_TAG ${YAML_CPP_TAG}
  PATCH_COMMAND tar -xzf ../yaml_cpp_src.tar.gz
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX} -DBUILD_SHARED_LIBS=ON
)

cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} yaml_cpp_src)
target_link_libraries(${PROJECT_NAME} ${CATKIN_DEVEL_PREFIX}/lib/libyaml-cpp0.5${CMAKE_SHARED_LIBRARY_SUFFIX})

cs_install()

cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  CFG_EXTRAS yaml-cpp-extras.cmake)
