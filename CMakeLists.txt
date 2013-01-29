##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.4.6)

##############################################################################
# Ros Initialisation
##############################################################################

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

##############################################################################
# compiler flags
##############################################################################

rosbuild_check_for_sse ()
#add_definitions (-Wall -O3 -Winvalid-pch -DNDEBUG -funroll-loops -fno-strict-aliasing -DEIGEN_NO_DEBUG)
add_definitions (-Wall -fno-strict-aliasing)
#add_definitions(-Wall -O3 -fno-strict-aliasing -funroll-loops -momit-leaf-frame-pointer -fomit-frame-pointer -floop-block -ftree-loop-distribution -ftree-loop-linear -floop-interchange -floop-strip-mine -DEIGEN_NO_DEBUG -DNDEBUG -fgcse-lm -fgcse-sm -fsched-spec-load)

##############################################################################
# Qt Environment
##############################################################################

find_package(Qt4 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Eigen REQUIRED)
set( QT_USE_QTOPENGL TRUE )
include(${QT_USE_FILE})

##############################################################################
# sources (StructureColoring)
##############################################################################

set( common_sources
  src/grids/GridMap.cpp
  src/grids/CylinderGridMap.cpp

  src/histograms/SphereUniformSampling.cpp

  src/structures/CylinderPatch.cpp
  src/structures/PlanePatch.cpp
  
  src/segcomp/rangeimageio.cpp
  src/segcomp/rangetocoords.cpp

  src/StrColParams.cpp
  src/RosVisualization.cpp
  src/StructureColoring.cpp
)

##############################################################################
# includes (StructureColoring)
##############################################################################

include_directories(
  ${CMAKE_BINARY_DIR}
  ${EIGEN_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR}
)

add_definitions(${EIGEN_DEFINITIONS})

##############################################################################
# libraries (StructureColoring)
##############################################################################

rosbuild_add_library(structureColoring ${common_sources} ${headers})
rosbuild_add_compile_flags (structureColoring ${SSE_FLAGS})
rosbuild_add_openmp_flags(structureColoring)

target_link_libraries(structureColoring
  ${QT_LIBRARIES}
  ${OpenCV_LIBS}
)

##############################################################################
# binaries (StructureColoring)
##############################################################################

rosbuild_add_executable(segmentSC src/Main.cpp)
rosbuild_add_compile_flags (segmentSC ${SSE_FLAGS})
rosbuild_add_openmp_flags(segmentSC)

target_link_libraries(segmentSC
  structureColoring
)

##############################################################################
# sources (StructureGL)
##############################################################################

set( StructureGLsources
  src/gui/StructureGL.cpp
)

##############################################################################
# headers (StructureGL)
##############################################################################

set( StructureGLheaders
  include/structureColoring/gui/StructureGL.h
)
qt4_wrap_cpp(moc_sources ${StructureGLheaders})

##############################################################################
# includes (StructureGL)
##############################################################################

include_directories(
  ${CMAKE_BINARY_DIR}
  ${QT_INCLUDE_DIR}
  ${QT_QTOPENGL_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR}
  ${VPCL2_INCLUDE_DIR}
)

##############################################################################
# binaries (StructureGL)
##############################################################################

rosbuild_add_executable(GuiMain src/GuiMain.cpp ${StructureGLsources} ${StructureGLheaders} ${moc_sources})
rosbuild_add_openmp_flags(GuiMain)

target_link_libraries(GuiMain
  structureColoring
  ${QT_LIBRARIES}
  ${QT_QTOPENGL_LIBRARIES}
  ${OPENGL_LIBRARIES}
)

##############################################################################
# libraries (StructureGL)
##############################################################################

rosbuild_add_library(structureGL ${StructureGLsources} ${StructureGLheaders} ${moc_sources})
rosbuild_add_compile_flags (structureGL ${SSE_FLAGS})
rosbuild_add_openmp_flags(structureGL)

target_link_libraries(structureGL
  structureColoring
  ${QT_LIBRARIES}
  ${QT_QTOPENGL_LIBRARIES}
  ${OPENGL_LIBRARIES}
)

##############################################################################
# convertToRas
##############################################################################

rosbuild_add_executable(convertToRas src/segcomp/convertToRas.cpp)
target_link_libraries(convertToRas
  structureColoring
  ${QT_LIBRARIES}
)
