project(KinectUtils)
cmake_minimum_required(VERSION 2.8)

set(KINECT_UTILS_VERSION 1.0)
message("Using Utilities v" ${KINECT_UTILS_VERSION})

if(KINECT_ENABLED)
  set(KINTECT_UTILS_SRC
    Backend/KinectUtils/KinectHandler.cpp
    Backend/KinectUtils/GePoTool.cpp
    Backend/KinectUtils/SkeletonSmoother.cpp
    Backend/SnapshotTaker.cpp
  )

  list(APPEND KINTECT_UTILS_HEADERS
    Backend/KinectUtils/KinectHandler.h
    Backend/KinectUtils/GePoTool.h
    Backend/KinectUtils/GestureDetectorBase.h
    Backend/KinectUtils/SkeletonSmoother.h
    Backend/SnapshotTaker.h
  )

#Append Gesture Detectors
  list(APPEND KINTECT_UTILS_SRC
    Backend/KinectUtils/LeanDetector.cpp
    Backend/KinectUtils/KickDetector.cpp
    Backend/KinectUtils/PunchDetector.cpp
    Backend/KinectUtils/JumpDetector.cpp
    Backend/KinectUtils/SwipeDetector.cpp
    Backend/KinectUtils/ArmWaveDetector.cpp
    Backend/KinectUtils/BrushDetector.cpp
    Backend/KinectUtils/HandZoomDetector.cpp
    Backend/KinectUtils/OpenArmDetector.cpp
    Backend/KinectUtils/HandUpDetector.cpp
    Backend/KinectUtils/TomDetector.cpp
    Backend/KinectUtils/SquatDetector.cpp
  )

  list(APPEND KINTECT_UTILS_HEADERS
    Backend/KinectUtils/LeanDetector.h
    Backend/KinectUtils/KickDetector.h
    Backend/KinectUtils/PunchDetector.h
    Backend/KinectUtils/JumpDetector.h
    Backend/KinectUtils/SwipeDetector.h
    Backend/KinectUtils/ArmWaveDetector.h
    Backend/KinectUtils/BrushDetector.h
    Backend/KinectUtils/HandZoomDetector.h
    Backend/KinectUtils/OpenArmDetector.h
    Backend/KinectUtils/HandUpDetector.h
    Backend/KinectUtils/TomDetector.h
    Backend/KinectUtils/SquatDetector.h
  )
else()
  message("Kinect is not enabled.")
endif(KINECT_ENABLED)

add_library(kinect_utils STATIC
  ${KINTECT_UTILS_HEADERS}
)

set_target_properties(kinect_utils
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

