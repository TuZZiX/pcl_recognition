cmake_minimum_required(VERSION 2.8.3)
project(pcl_recognition)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next line to use the point-cloud library
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

set(CMAKE_BUILD_TYPE Release)

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(pcl_lib src/pcl_utils.cpp)
cs_add_library(object_recognizer src/object_recognizer.cpp)

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
#cs_add_executable(pcl_recognition src/pcl_recognition.cpp)
cs_add_executable(correspondence_grouping src/correspondence_grouping.cpp)
cs_add_executable (cluster_extraction src/cluster_extraction.cpp)
cs_add_executable (find_stool_coke src/find_plane_pcd_file.cpp)
#cs_add_executable (remove_outliers src/remove_outliers.cpp)
cs_add_executable (implicit_shape_model src/implicit_shape_model.cpp)
cs_add_executable (icp src/ICP.cpp)
cs_add_executable (global_hypothesis_verification src/global_hypothesis_verification.cpp)
cs_add_executable (pcd_edit_tool src/pcd_edit_tool.cpp)
cs_add_executable (object_recognize_main src/object_recognize_main.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
#target_link_libraries(pcl_recognition pcl_basic)
target_link_libraries (correspondence_grouping ${PCL_LIBRARIES})
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})
target_link_libraries (find_stool_coke pcl_lib ${PCL_LIBRARIES})
#target_link_libraries (remove_outliers ${PCL_LIBRARIES})
target_link_libraries (implicit_shape_model ${PCL_LIBRARIES})
target_link_libraries (icp pcl_lib ${PCL_LIBRARIES})
target_link_libraries (global_hypothesis_verification ${PCL_LIBRARIES})
target_link_libraries (pcd_edit_tool pcl_lib ${PCL_LIBRARIES})
target_link_libraries (object_recognize_main object_recognizer ${PCL_LIBRARIES})

cs_install()
cs_export()
    
