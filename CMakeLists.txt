project(KinectUtils)
cmake_minimum_required(VERSION 2.8)

include(FindKinectSDK2.cmake)
set(KINECT_UTILS_VERSION 1.0)
message("Using KinectUtils v" ${KINECT_UTILS_VERSION})

include_directories(${KinectSDK2_INCLUDE_DIRS})

if(KinectSDK2_FOUND)
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
  message("Kinect is not enabled.")
endif(KinectSDK2_FOUND)

add_library(kinect_utils STATIC
  ${KINTECT_UTILS_SRC}
)

set_target_properties(kinect_utils
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

