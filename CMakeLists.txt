project(Kinectron)
cmake_minimum_required(VERSION 2.8)
set(KINECTRON_VERSION 1.0)
message("NOTICE [Kinectron]: Using Kinectron v" ${KINECTRON_VERSION})

# Try to find Kinect
if(AR STREQUAL "x64" OR AR STREQUAL "x86")
    message("NOTICE [Kinectron]: Using libraries for ${AR}")
else()
    set(AR x86)
    message("NOTICE [Kinectron]: Architecture is not set. Defaulting to x86")
endif()

if (KINECTRON_SINGLE_THREAD)
    add_definitions(-DKINECTRON_SINGLE_THREAD=1)
    message("NOTICE [Kinectron]: Using the main thread for the update method. To enable multi-threading set KINECTRON_SINGLE_THREAD to false")
else()
    add_definitions(-DKINECTRON_SINGLE_THREAD=0)
    message("NOTICE [Kinectron]: Using a seperate thread for the update method")
endif(KINECTRON_SINGLE_THREAD)

set(KINECT_ENABLED OFF CACHE BOOL "Kinect 2.x SDK found")
set(KINECTSDK20_DIR "NOT FOUND" CACHE PATH "Kinect 2.x SDK path")
if(WIN32)
    if(EXISTS $ENV{KINECTSDK20_DIR})
        set(KINECT_ENABLED ON CACHE BOOL "Kinect 2.x SDK found" FORCE)
        set(KinectSDK2_DIR $ENV{KinectSDK2_DIR} CACHE PATH "Kinect 2.x SDK path" FORCE)

        if(EXISTS $ENV{KINECTSDK20_DIR}/inc)
            set(KinectSDK2_INCLUDE_DIRS $ENV{KINECTSDK20_DIR}/inc CACHE PATH "Kinect v2 inclue path" FORCE)
        else()
            message("ERROR [Kinectron]: Kinect includes files cannot be found!")
        endif()
        if(EXISTS $ENV{KINECTSDK20_DIR}/lib/${AR}/)
            set(KinectSDK2_LIBRARY_DIRS $ENV{KINECTSDK20_DIR}/lib/${AR}/ CACHE PATH "Kinect v2 library dirs" FORCE)
        else()
            message("ERROR [Kinectron]: Kinect ${AR} library dir cannot be found")
        endif()
        if(EXISTS $ENV{KINECTSDK20_DIR}/lib/${AR}/Kinect20.lib)
            set(KINECTRON_LIB $ENV{KINECTSDK20_DIR}/lib/${AR}/Kinect20.lib CACHE PATH "Kinect v2 library path" FORCE)
        else()
            message("ERROR [Kinectron]: Kinect ${AR} library file cannot be found")
        endif()
    endif()
else()
endif()

get_filename_component(KINECTRON_ROOT "./" ABSOLUTE)

set(KINECTRON_INC
    ${KINECTRON_ROOT}
    ${KinectSDK2_INCLUDE_DIRS}
    ${KINECTRON_ROOT}/Detectors
    CACHE LIST "Set the list of include paths for Kinectron")

include_directories(${KINECTRON_INC})

if(KINECT_ENABLED)
  set(KINECTRON_SRC
    KinectHandler.cpp
    GePoTool.cpp
    SkeletonSmoother.cpp
  )

  list(APPEND KINECTRON_HEADERS
    KinectHandler.h
    GePoTool.h
    Detectors/GestureDetectorBase.h
    SkeletonSmoother.h
    KinectronTypes.h
  )

#Append Gesture Detectors
  list(APPEND KINECTRON_SRC
    Detectors/LeanDetector.cpp
    Detectors/KickDetector.cpp
    Detectors/PunchDetector.cpp
    Detectors/JumpDetector.cpp
    Detectors/SwipeDetector.cpp
    Detectors/ArmWaveDetector.cpp
    Detectors/BrushDetector.cpp
    Detectors/HandZoomDetector.cpp
    Detectors/OpenArmDetector.cpp
    Detectors/HandUpDetector.cpp
    Detectors/TomDetector.cpp
    Detectors/SquatDetector.cpp
  )

  list(APPEND KINECTRON_HEADERS
    Detectors/LeanDetector.h
    Detectors/KickDetector.h
    Detectors/PunchDetector.h
    Detectors/JumpDetector.h
    Detectors/SwipeDetector.h
    Detectors/ArmWaveDetector.h
    Detectors/BrushDetector.h
    Detectors/HandZoomDetector.h
    Detectors/OpenArmDetector.h
    Detectors/HandUpDetector.h
    Detectors/TomDetector.h
    Detectors/SquatDetector.h
  )
else()
    message("ERROR [Kinectron]: Kinect SDK cannot be found so it is not enabled.")
endif(KINECT_ENABLED)

add_library(kinectron STATIC
  ${KINECTRON_SRC}
)

set_target_properties(kinectron
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)
