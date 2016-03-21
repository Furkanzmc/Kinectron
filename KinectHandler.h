#ifndef KINECTHANDLER_H
#define KINECTHANDLER_H
#include "KinectUtilsTypes.h"

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

    static const unsigned int BITS_PER_PIXEL_COLOR = sizeof(RGBQUAD) * 8;
    static const unsigned int COLOR_WIDTH = 1920;
    static const unsigned int COLOR_HEIGHT = 1080;
    static const unsigned int DATA_LENGTH_COLOR = COLOR_WIDTH * COLOR_HEIGHT * (BITS_PER_PIXEL_COLOR / 8);

    static const unsigned int BITS_PER_PIXEL_DEPTH = sizeof(RGBTRIPLE) * 8;
    static const unsigned int DEPTH_WIDTH = 512;
    static const unsigned int DEPTH_HEIGHT = 424;
    static const unsigned int DATA_LENGTH_DEPTH = DEPTH_WIDTH * DEPTH_HEIGHT * (BITS_PER_PIXEL_DEPTH / 8);

    static const unsigned int BITS_PER_PIXEL_BODY_INDEX = BITS_PER_PIXEL_DEPTH;
    static const unsigned int BODY_INDEX_WIDTH = DEPTH_WIDTH;
    static const unsigned int BODY_INDEX_HEIGHT = DEPTH_HEIGHT;
    static const unsigned int DATA_LENGTH_BODY_INDEX = DATA_LENGTH_DEPTH;

    static const unsigned int BITS_PER_PIXEL_IR = BITS_PER_PIXEL_DEPTH;
    static const unsigned int IR_WIDTH = DEPTH_WIDTH;
    static const unsigned int IR_HEIGHT = DEPTH_HEIGHT;
    static const unsigned int DATA_LENGTH_IR = DATA_LENGTH_DEPTH;

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

    const unsigned short *getDepthData() const;
    bool isDepthDataAvailable() const;

    const unsigned short *getBodyIndexData() const;
    bool isBodyIndexDataAvailable() const;

    const unsigned short *getIRData() const;
    bool isIRDataAvailable() const;

    PointF mapBodyPointToScreenPoint(const CameraSpacePoint &bodyPoint);

    const UINT64 &getClosestBodyID() const;

    /**
     * @brief Any skeleton that is behind the closest skeleton by this offset is deleted. If it equals to -1 this removing is not done.
     * @param offset
     */
    void setClosestBodyOffset(const float &offset);
    const float &getClosestBodyOffset() const;

    unsigned int getDesiredBodyCount() const;
    void setDesiredBodyCount(unsigned int desiredBodyCount);

    void setIRSourceValueMax(float maxVal);
    float getIRSourceValueMax() const;

    void setIROutputValueMin(float minVal);
    float getIROutputValueMin() const;

    void setIROutputValueMax(float maxVal);
    float getIROutputValueMax() const;

    void setIRSceneValueAvg(float avgVal);
    float getIRSceneValueAvg() const;

    void setIRSceneStandartDeviations(float deviation);
    float getIRSceneStandartDeviations() const;

private:
    bool m_isColorDataAvailable,// This is set to true when the Kinect color processing is done
         m_isDepthDataAvailable,// This is set to true when the Kinect depth processing is done
         m_isBodyIndexDataAvailable,// This is set to true when the Kinect body index processing is done
         m_isIRDataAvailable,// This is set to true when the Kinect infrred processing is done
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
    IInfraredFrame *m_IRFrame;

    ICoordinateMapper *m_CoordinateMapper;

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

    DepthFrameInfo m_DepthInfo;
    ColorFrameInfo m_ColorFrameInfo;
    BodyIndexInfo m_BodyIndexInfo;
    IRFrameInfo m_IRInfo;

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
    HRESULT updateIRFrameData(IRFrameInfo &irFrameInfo, IInfraredFrame *irFrame);
};

#endif // KINECTHANDLER_H
