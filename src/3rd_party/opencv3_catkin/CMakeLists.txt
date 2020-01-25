cmake_minimum_required(VERSION 2.8.3)
project(opencv3_catkin)

find_package(catkin_simple REQUIRED)

catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

SET(CONTRIB_NAME "opencv3_contrib")
ExternalProject_Add(${CONTRIB_NAME}
  URL ${CMAKE_SOURCE_DIR}/opencv3_contrib_src.tar.gz
  SOURCE_DIR ${CONTRIB_NAME}_src
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

SET(OPENCV_SRC_PATH "opencv3_src")
ExternalProject_Add(opencv3_src
  DEPENDS ${CONTRIB_NAME}
  URL ${CMAKE_SOURCE_DIR}/opencv-3.2.0.zip
  UPDATE_COMMAND ""
  SOURCE_DIR ${OPENCV_SRC_PATH}
  BINARY_DIR opencv3_build
  PATCH_COMMAND patch -p0 < ${CMAKE_SOURCE_DIR}/fix_python_discovery.patch &&
                patch -p0 < ${CMAKE_SOURCE_DIR}/fix_lapack.patch
  CONFIGURE_COMMAND cd ../${OPENCV_SRC_PATH} && cmake
    -DOPENCV_EXTRA_MODULES_PATH=../${CONTRIB_NAME}_src/modules .
    -DWITH_TBB=ON
    -DBUILD_opencv_python2=ON
    -DBUILD_opencv_python3=OFF
    -DWITH_GTK_2_X=ON  # Can't use GTK3 as it links against system protobuf.
    -DWITH_V4L=ON
    -DINSTALL_C_EXAMPLES=OFF
    -DINSTALL_PYTHON_EXAMPLES=OFF
    -DBUILD_EXAMPLES=OFF
    -DWITH_QT=OFF # Needed by clang under Ubuntu 14.04 and GTK_WIDGET(cvGetWindowHandle(...)) with gcc (image_view)
    -DWITH_OPENGL=ON
    -DWITH_VTK=ON
    -DENABLE_PRECOMPILED_HEADERS=OFF
    -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
    -DBUILD_SHARED_LIBS=ON
    -DWITH_CUDA=OFF
    -DWITH_OPENCL=OFF
    -DWITH_IPP=OFF
    -DBUILD_TESTS=OFF
    -DBUILD_opencv_ts=OFF
    -DBUILD_TESTS=OFF
    # opencv_contrib packages
    -DBUILD_opencv_dnn=OFF # Pulls in the system protobuf as a dependency!
    -DBUILD_opencv_dnns_easily_fooled=OFF
    -DBUILD_opencv_cnn_3dobj=OFF
    -DBUILD_opencv_aruco=OFF
    -DBUILD_opencv_bgsegm=OFF
    -DBUILD_opencv_bioinspired=OFF
    -DBUILD_opencv_ccalib=OFF
    -DBUILD_opencv_contrib_world=OFF
    -DBUILD_opencv_datasets=OFF
    -DBUILD_opencv_dpm=OFF
    -DBUILD_opencv_face=OFF
    -DBUILD_opencv_fuzzy=OFF
    -DBUILD_opencv_freetype=OFF
    -DBUILD_opencv_hdf=OFF
    -DBUILD_opencv_line_descriptor=OFF
    -DBUILD_opencv_matlab=OFF
    -DBUILD_opencv_optflow=OFF
    -DBUILD_opencv_plot=OFF
    -DBUILD_opencv_reg=OFF
    -DBUILD_opencv_rgbd=ON
    -DBUILD_opencv_saliency=ON
    -DBUILD_opencv_sfm=OFF
    -DBUILD_opencv_stereo=OFF
    -DBUILD_opencv_structured_light=OFF
    -DBUILD_opencv_surface_matching=OFF
    -DBUILD_opencv_text=OFF
    -DBUILD_opencv_tracking=OFF
    -DBUILD_opencv_xfeatures2d=ON
    -DBUILD_opencv_ximgproc=OFF
    -DBUILD_opencv_xobjdetect=OFF
    -DBUILD_opencv_xphoto=OFF .
  BUILD_COMMAND cd ../${OPENCV_SRC_PATH} && make -j8
  INSTALL_COMMAND cd ../${OPENCV_SRC_PATH} && make install -j8
)

cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} opencv3_src)
target_link_libraries(${PROJECT_NAME}
  ${CATKIN_DEVEL_PREFIX}/lib/libopencv_core${CMAKE_SHARED_LIBRARY_SUFFIX})

cs_install()

cs_export(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
CFG_EXTRAS opencv3-extras.cmake)
