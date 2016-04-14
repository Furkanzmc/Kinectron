#ifndef KINECTUTILTYPES_H
#define KINECTUTILTYPES_H
//Windows Includes
#include <Windows.h>
#include <strsafe.h>
#include <Shlobj.h>
#include <Objbase.h>
#pragma comment(lib, "shell32.lib")
#pragma comment(lib, "Ole32.lib")

//Kinect Includes
#include "Kinect.h"

//STD includes
#include <functional>
#include <thread>
#include <mutex>
#include <vector>
#include <array>

/**
 * The index from 0 to 5 each corresponding to a skeleton. Player 0 is the left-most player, and player 5 is the right most player
 */
using BodyIndex = unsigned int;

template<class Interface>
static inline void safeRelease(Interface *&interfaceToRelease)
{
    if (interfaceToRelease != nullptr) {
        interfaceToRelease->Release();
        interfaceToRelease = nullptr;
    }
}

struct DepthFrameInfo {
    ~DepthFrameInfo()
    {
        safeRelease(frameDescription);

        if (bufferRGB) {
            delete[] bufferRGB;
            bufferRGB = nullptr;
        }
    }

    void reset()
    {
        time = 0;
        safeRelease(frameDescription);
        width = 0;
        height = 0;
        minReliableDistance = 0;
        maxReliableDistance = 0;
        bufferSize = 0;
        buffer = nullptr;
        if (bufferRGB) {
            delete[] bufferRGB;
            bufferRGB = nullptr;
        }
    }

    INT64 time = 0;
    IFrameDescription *frameDescription = nullptr;
    int width = 0;
    int height = 0;
    USHORT minReliableDistance = 0, maxReliableDistance = 0;
    UINT bufferSize = 0;
    UINT16 *buffer = nullptr;
    RGBTRIPLE *bufferRGB = nullptr;
};

struct ColorFrameInfo {
    ~ColorFrameInfo()
    {
        safeRelease(frameDescription);
        if (bufferRGBX) {
            delete[] bufferRGBX;
            bufferRGBX = nullptr;
        }
    }

    void reset()
    {
        time = 0;
        safeRelease(frameDescription);
        width = 0;
        height = 0;
        imageFormat = ColorImageFormat_None;
        bufferSize = 0;
        if (bufferRGBX) {
            delete[] bufferRGBX;
            bufferRGBX = nullptr;
        }
    }

    INT64 time = 0;
    IFrameDescription *frameDescription = nullptr;
    int width = 0;
    int height = 0;
    ColorImageFormat imageFormat = ColorImageFormat_None;
    UINT bufferSize = 0;
    RGBQUAD *bufferRGBX = nullptr;
};

struct BodyIndexFrameInfo {
    ~BodyIndexFrameInfo()
    {
        safeRelease(frameDescription);
        if (bufferRGB) {
            delete[] bufferRGB;
            bufferRGB = nullptr;
        }
    }

    void reset()
    {
        time = 0;
        safeRelease(frameDescription);
        width = 0;
        height = 0;
        bufferSize = 0;
        buffer = nullptr;
        if (bufferRGB) {
            delete[] bufferRGB;
            bufferRGB = nullptr;
        }
    }

    INT64 time = 0;
    IFrameDescription *frameDescription = nullptr;
    int width = 0;
    int height = 0;
    UINT bufferSize = 0;
    UINT8 *buffer = nullptr;
    RGBTRIPLE *bufferRGB = nullptr;
};

struct BodyFrameInfo {
    INT64 time = 0;
    Vector4 floorClipPlane;

    void reset()
    {
        time = 0;
        floorClipPlane = { 0, 0, 0, 0 };
    }
};

struct IRFrameInfo {
    ~IRFrameInfo()
    {
        safeRelease(frameDescription);

        if (bufferRGB) {
            delete[] bufferRGB;
            bufferRGB = nullptr;
        }
    }

    void reset()
    {
        time = 0;
        safeRelease(frameDescription);
        width = 0;
        height = 0;
        minReliableDistance = 0;
        maxReliableDistance = 0;
        bufferSize = 0;
        buffer = nullptr;
        sourceValueMaximum = static_cast<float>(USHRT_MAX);
        outputValueMinimum = 0.01f;
        outputValueMaximum = 1.0f;
        sceneValueAverage = 0.08f;
        sceneStandardDeviations = 3.0f;
        if (bufferRGB) {
            delete[] bufferRGB;
            bufferRGB = nullptr;
        }
    }

    INT64 time = 0;
    IFrameDescription *frameDescription = nullptr;
    int width = 0;
    int height = 0;
    USHORT minReliableDistance = 0, maxReliableDistance = 0;
    UINT bufferSize = 0;
    UINT16 *buffer = nullptr;
    RGBTRIPLE *bufferRGB = nullptr;

    // InfraredSourceValueMaximum is the highest value that can be returned in the InfraredFrame.
    // It is cast to a float for readability in the visualization code.
    float sourceValueMaximum = static_cast<float>(USHRT_MAX);

    // The InfraredOutputValueMinimum value is used to set the lower limit, post processing, of the
    // infrared data that we will render.
    // Increasing or decreasing this value sets a brightness "wall" either closer or further away.
    float outputValueMinimum = 0.01f;

    // The InfraredOutputValueMaximum value is the upper limit, post processing, of the
    // infrared data that we will render.
    float outputValueMaximum = 1.0f;

    // The InfraredSceneValueAverage value specifies the average infrared value of the scene.
    // This value was selected by analyzing the average pixel intensity for a given scene.
    // Depending on the visualization requirements for a given application, this value can be
    // hard coded, as was done here, or calculated by averaging the intensity for each pixel prior
    // to rendering.
    float sceneValueAverage = 0.08f;

    // The InfraredSceneStandardDeviations value specifies the number of standard deviations
    // to apply to InfraredSceneValueAverage. This value was selected by analyzing data
    // from a given scene.
    // Depending on the visualization requirements for a given application, this value can be
    // hard coded, as was done here, or calculated at runtime.
    float sceneStandardDeviations = 3.0f;
};

#endif // KINECTUTILTYPES_H
