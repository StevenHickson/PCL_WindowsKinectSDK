cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCL_WindowsKinectSDK)

add_subdirectory(src)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(C:/blepo/external64/Microsoft/Kinect)
include_directories(${PCL_WindowsKinectSDK_SOURCE_DIR}/include)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(C:/blepo/external64/Microsoft/Kinect)

add_definitions(${PCL_DEFINITIONS})

file(GLOB_RECURSE PCL_WindowsKinectSDK_HEADERS include/*.h)
file(GLOB_RECURSE PCL_WindowsKinectSDK_SOURCES src/*.cpp)
set (PCL_WindowsKinectSDK_INCLUDE_DIRS "")
foreach (_headerFile ${PCL_WindowsKinectSDK_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND PCL_WindowsKinectSDK_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES PCL_WindowsKinectSDK_INCLUDE_DIRS)

include_directories(${PCL_WindowsKinectSDK_INCLUDE_DIRS})
add_executable (PCL_WindowsKinectSDK ${PCL_WindowsKinectSDK_SOURCES})
target_link_libraries (PCL_WindowsKinectSDK ${PCL_LIBRARIES} Kinect10.lib)