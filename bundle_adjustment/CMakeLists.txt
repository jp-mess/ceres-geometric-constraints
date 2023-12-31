cmake_minimum_required(VERSION 3.0)
project(CeresGeometricConstraints)

# Find Ceres and specify its location
find_package(Ceres REQUIRED)

# Set Glog include directory
set(GLOG_INCLUDE_DIR "/usr/local/include/glog/") # Set to your Glog include directory

# Add the executable for bundle adjustment
add_executable(bundle_adjust bundle_adjust.cc)

# Add the executable for assessing reprojection error
add_executable(assess_reprojection assess_reprojection.cc)

# Add the executable for the ring test
add_executable(ring_test ring_test.cc)

# Include directories for Glog
target_include_directories(bundle_adjust PUBLIC ${GLOG_INCLUDE_DIR})
target_include_directories(assess_reprojection PUBLIC ${GLOG_INCLUDE_DIR})
target_include_directories(ring_test PUBLIC ${GLOG_INCLUDE_DIR})

# Link Ceres and Glog to your executables
target_link_libraries(bundle_adjust Ceres::ceres /usr/local/lib/libglog.so)
target_link_libraries(assess_reprojection Ceres::ceres /usr/local/lib/libglog.so)
target_link_libraries(ring_test Ceres::ceres /usr/local/lib/libglog.so)
