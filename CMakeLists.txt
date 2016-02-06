project(KinectUtils)
cmake_minimum_required(VERSION 2.8)
set(KINECT_UTILS_VERSION 1.0)
message("NOTICE [KinectUtils]: Using KinectUtils v" ${KINECT_UTILS_VERSION})

# Try to find Kinect
if(AR STREQUAL "x64" OR AR STREQUAL "x86")
    message("NOTICE [KinectUtils]: Using libraries for ${AR}")
else()
    set(AR x86)
    message("NOTICE [KinectUtils]: Architecture is not set. Defaulting to x86")
endif()

set(KINECT_ENABLED OFF CACHE BOOL "Kinect 2.x SDK found")
set(KINECTSDK20_DIR "NOT FOUND" CACHE PATH "Kinect 2.x SDK path")
if(WIN32)
    if(EXISTS $ENV{KINECTSDK20_DIR})
        set(KINECT_ENABLED ON CACHE BOOL "Kinect 2.x SDK found" FORCE)
        set(KinectSDK2_DIR $ENV{KinectSDK2_DIR} CACHE PATH "Kinect 2.x SDK path" FORCE)

        if(EXISTS $ENV{KINECTSDK20_DIR}/inc)
            set(KinectSDK2_INCLUDE_DIRS $ENV{KINECTSDK20_DIR}/inc CACHE PATH "Kinect v2 inclue path" FORCE)
        else()
            message("ERROR [KinectUtils]: Kinect includes files cannot be found!")
        endif()
        if(EXISTS $ENV{KINECTSDK20_DIR}/lib/${AR}/)
            set(KinectSDK2_LIBRARY_DIRS $ENV{KINECTSDK20_DIR}/lib/${AR}/ CACHE PATH "Kinect v2 library dirs" FORCE)
        else()
            message("ERROR [KinectUtils]: Kinect ${AR} library dir cannot be found")
        endif()
        if(EXISTS $ENV{KINECTSDK20_DIR}/lib/${AR}/Kinect20.lib)
            set(KinectSDK2_LIBRARIES $ENV{KINECTSDK20_DIR}/lib/${AR}/Kinect20.lib CACHE PATH "Kinect v2 library path" FORCE)
        else()
            message("ERROR [KinectUtils]: Kinect ${AR} library file cannot be found")
        endif()
    endif()
else()
endif()

include_directories(${KinectSDK2_INCLUDE_DIRS})

if(KINECT_ENABLED)
  set(KINTECT_UTILS_SRC
    KinectHandler.cpp
    GePoTool.cpp
    SkeletonSmoother.cpp
  )

  list(APPEND KINTECT_UTILS_HEADERS
    KinectHandler.h
    GePoTool.h
    GestureDetectorBase.h
    SkeletonSmoother.h
  )

#Append Gesture Detectors
  list(APPEND KINTECT_UTILS_SRC
    LeanDetector.cpp
    KickDetector.cpp
    PunchDetector.cpp
    JumpDetector.cpp
    SwipeDetector.cpp
    ArmWaveDetector.cpp
    BrushDetector.cpp
    HandZoomDetector.cpp
    OpenArmDetector.cpp
    HandUpDetector.cpp
    TomDetector.cpp
    SquatDetector.cpp
  )

  list(APPEND KINTECT_UTILS_HEADERS
    LeanDetector.h
    KickDetector.h
    PunchDetector.h
    JumpDetector.h
    SwipeDetector.h
    ArmWaveDetector.h
    BrushDetector.h
    HandZoomDetector.h
    OpenArmDetector.h
    HandUpDetector.h
    TomDetector.h
    SquatDetector.h
  )
else()
    message("ERROR [KinectUtils]: Kinect SDK cannot be found so it is not enabled.")
endif(KINECT_ENABLED)

add_library(kinect_utils STATIC
  ${KINTECT_UTILS_SRC}
)

set_target_properties(kinect_utils
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

