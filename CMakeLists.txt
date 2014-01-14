cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)

IF(WIN32 OR win64)
   SET(CMAKE_FIND_LIBRARY_SUFFIXES .lib .dll)
ELSE()
   SET(CMAKE_FIND_LIBRARY_SUFFIXES .a)
ENDIF()

find_package(PCL 1.7 REQUIRED)
find_package( OpenCV REQUIRED )

project(PCL_WindowsKinectSDK)

add_subdirectory(src)

include_directories(C:/opencv/build)
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
add_library(PCL_WindowsKinectSDK STATIC src/Microsoft_grabber.cpp)
target_link_libraries (PCL_WindowsKinectSDK ${PCL_LIBRARIES} ${OpenCV_LIBS} Kinect10.lib)

add_executable (Test_SDK src/PCL_WindowsKinectSDK.cpp)
target_link_libraries (Test_SDK PCL_WindowsKinectSDK ${PCL_LIBRARIES} ${OpenCV_LIBS} Kinect10.lib)