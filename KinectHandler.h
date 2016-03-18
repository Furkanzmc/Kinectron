#ifndef KINECTHANDLER_H
#define KINECTHANDLER_H
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

class KinectHandler
{
public:
    /**
     * @brief This is called when the takeSanpshot() method is called. Take the data and run the function you want
     * in OpenGL thread
     * Parameters are, in order, color data, data lenghts, color width, color height, bits per pixel, file path
     */
    std::function<void(const unsigned char *, const unsigned int &, const unsigned int &, const unsigned int &, const unsigned int &, const std::string &)>
    m_TakeScreenshotFunc;

    /**
     * @brief Bodies are provided in a sorted way. Bodies are sorted from left to right on the X-axis. Take the data and run the function you want
     * in OpenGL thread
     * Parameters are body, sensor time, visible body count, index.
     */
    std::function<void(const std::array<IBody *, BODY_COUNT>&, UINT64)> m_ProcessBodyFunc;
    std::function<void()> m_ResetBodyFunc;

    static const unsigned int BITS_PER_PIXEL = sizeof(RGBQUAD) * 8;
    static const unsigned int COLOR_WIDTH = 1920;
    static const unsigned int COLOR_HEIGHT = 1080;
    static const unsigned int DATA_LENGTH = COLOR_WIDTH * COLOR_HEIGHT * (BITS_PER_PIXEL / 8);
    static const unsigned int DEPTH_WIDTH = 512;
    static const unsigned int DEPTH_HEIGHT = 424;

public:
    KinectHandler();
    ~KinectHandler();

    /**
     * @brief Use bitmasking to initizlize more than one source.
     * @param initeType - FrameSourceTypes bit masking
     * @return Returns the success of the operation as HRESULT
     */
    HRESULT initializeDefaultSensor(const unsigned int &initeType);
    bool isKinectAvailable();

    /**
     * @brief Closes the sensor and stops the update thread.
     * @return
     */
    HRESULT closeSensor();

    /**
     * @brief Calls the m_TakeScreenshotFunc function in a different thread with the given file path
     * @param filePath
     */
    void takeSanpshot(const std::string &filePath);

    const Vector4 &getFloorClipPlane();
    bool isFloorVisible() const;
    double getDistanceFromFloor(const CameraSpacePoint &jointPosition) const;

    ICoordinateMapper *getCoordinateMapper() const;

    const unsigned char *getColorData() const;
    bool isColorDataAvailable() const;

    PointF mapBodyPointToScreenPoint(const CameraSpacePoint &bodyPoint);

    const UINT64 &getClosestBodyID() const;

    /**
     * @brief Any skeleton that is behind the closest skeleton by this offset is deleted. If it equals to -1 this removing is not done.
     * @param offset
     */
    void setClosestBodyOffset(const float &offset);
    const float &getClosestBodyOffset() const;

    template<class Interface>
    static inline void safeRelease(Interface *&interfaceToRelease)
    {
        if (interfaceToRelease != nullptr) {
            interfaceToRelease->Release();
            interfaceToRelease = nullptr;
        }
    }

    unsigned int getDesiredBodyCount() const;
    void setDesiredBodyCount(unsigned int desiredBodyCount);

private:
    bool m_isColorDataAvailable,//This is set to true when the Kinect color processing is done
         m_CanTakeSnapshot,
         m_IsSensorClosed;
    std::string m_SnapshotFilePath;

    IKinectSensor *m_Sensor;
    IMultiSourceFrameReader *m_MultiSourceFrameReader;
    IMultiSourceFrame *m_MultiSourceFrame;
    IDepthFrame *m_DepthFrame;
    IColorFrame *m_ColorFrame;
    IBodyIndexFrame *m_BodyIndexFrame;
    IBodyFrame *m_BodyFrame;
    ICoordinateMapper *m_CoordinateMapper;
    DepthSpacePoint *m_DepthCoordinates;

    RGBQUAD *m_ColorRGBX;

    FrameSourceTypes m_InitType;
    Vector4 m_FloorClipPlane;
    WAITABLE_HANDLE m_FrameArrivedHandle;

    std::thread m_ThreadUpdate, m_ThreadScreenshot;
    std::recursive_mutex m_Mutex;

    UINT64 m_ClosestBodyID;
    /**
     * @brief Any skeleton that is behind the closest skeleton by this offset is deleted. If it equals to -1 this removing is not done.
     */
    float m_ClosestBodyOffset;

    /**
     * @brief The default value is 6 (BODY_COUNT). This cannot be 0, or bigger than 6. When this is smaller than 6, KinectHandler will first rule out the bodies based on m_ClosestBodyOffset.
     * Then, it will get the closest bodies to the center of the sensor. And then, rule out the bodies based on the desired body count.
     * Left and right are based on screen, not the sensor. If the desired body count is 0, only the closest skeleton to the center is given.
     * Here's an example:
     * Say the desired body count is 2. Then, KinectHandler (KH) searches for the closest bodies on either side of the sensor. If it finds two skeletons
     * nothing is done. If it finds more, say there is one skeleton on the left and two on the right, it takes the one on the left. then it gets the closest skeleton to the center
     * on the right.
     */
    unsigned int m_DesiredBodyCount;

    struct DepthFrameInfo {
        ~DepthFrameInfo()
        {
            safeRelease(depthFrameDescription);
            if (depthBuffer) {
                delete[] depthBuffer;
                depthBuffer = nullptr;
            }
        }

        INT64 depthTime = 0;
        IFrameDescription *depthFrameDescription = nullptr;
        int depthWidth = 0;
        int depthHeight = 0;
        UINT depthBufferSize = 0;
        UINT16 *depthBuffer = nullptr;
    };

    struct ColorFrameInfo {
        ~ColorFrameInfo()
        {
            safeRelease(colorFrameDescription);
            if (colorBuffer) {
                delete[] colorBuffer;
                colorBuffer = nullptr;
            }
        }

        IFrameDescription *colorFrameDescription = nullptr;
        int colorWidth = 0;
        int colorHeight = 0;
        ColorImageFormat imageFormat = ColorImageFormat_None;
        UINT colorBufferSize = 0;
        RGBQUAD *colorBuffer = nullptr;
    };

    struct BodyIndexInfo {
        ~BodyIndexInfo()
        {
            safeRelease(bodyIndexFrameDescription);
            if (bodyIndexBuffer) {
                delete[] bodyIndexBuffer;
                bodyIndexBuffer = nullptr;
            }
        }

        IFrameDescription *bodyIndexFrameDescription = nullptr;
        int bodyIndexWidth = 0;
        int bodyIndexHeight = 0;
        UINT bodyIndexBufferSize = 0;
        UINT8 *bodyIndexBuffer = nullptr;
    };

    DepthFrameInfo m_DepthInfo;
    ColorFrameInfo m_ColorFrameInfo;
    BodyIndexInfo m_BodyIndexInfo;

private:
    /**
     * @brief Updates the sensor 30 times per second.
     */
    void updateSensor();

    /**
     * @brief Sorts the bodies from left to right and then feeds them to m_ProcessBodyFunc
     * @param delta
     * @param bodyCount
     * @param bodies
     */
    void processBody(UINT64 delta, int bodyCount, IBody **bodies);
    void processDesiredBodyCount(std::array<IBody *, BODY_COUNT> &visibleBodies);

    HRESULT updateDepthFrameData(DepthFrameInfo &depthInfo, IDepthFrame *depthFrame);
    HRESULT updateColorFrameData(ColorFrameInfo &colorFrameInfo, IColorFrame *colorFrame);
    HRESULT updateBodyIndexFrameData(BodyIndexInfo &bodyIndexInfo, IBodyIndexFrame *bodyIndexFrame);
    HRESULT updateBodyFrame(IBodyFrame *bodyFrame);
};

#endif // KINECTHANDLER_H
