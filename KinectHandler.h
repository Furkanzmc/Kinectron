#ifndef KINECTHANDLER_H
#define KINECTHANDLER_H
#include "KinectUtilsTypes.h"
#include <map>

class KinectHandler
{
public:
    /**
     * @brief This is called when the takeSanpshot() method is called and the color data is available.
     * This requires color stream. Take the data and run the function you want in main thread.
     * @param colorData
     * @param dataLength
     * @param width
     * @param height
     * @param bitsPerPixel
     */
    std::function<void(const unsigned char *colorData,
                       const unsigned int &dataLength,
                       const unsigned int &width,
                       const unsigned int &height,
                       const unsigned int &bitsPerPixel,
                       const std::string &)> onTakeScreenshot;

    /**
     * @brief Bodies are provided in a sorted way. Bodies are sorted from left to right on the X-axis. Take the data and run the function you want
     * in main thread
     * Parameters are body, sensor time, visible body count, index.
     */
    std::function<void(const std::array<IBody *, BODY_COUNT>&, UINT64)> onProcessBody;

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

    /** Sensor Functions **/

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
     * @brief Closes the sensor, terminates the threads, releases the frames and resets the frame infos
     */
    void reset();

    /**
     * @brief Calls the onTakeScreenshot function in a different thread with the given file path
     * @param filePath
     */
    void takeSnapshot(const std::string &filePath);

    const Vector4 &getFloorClipPlane();
    bool isFloorVisible() const;
    double getDistanceFromFloor(const CameraSpacePoint &jointPosition) const;

    ICoordinateMapper *getCoordinateMapper() const;

    PointF mapBodyPointToScreenPoint(const CameraSpacePoint &bodyPoint);

    unsigned int getInitType() const;

    /** Stream Getters **/

    const unsigned char *getColorData() const;
    const RGBQUAD *getColorDataRGB() const;
    bool isColorDataAvailable() const;

    const unsigned short *getDepthData() const;
    bool isDepthDataAvailable() const;

    const unsigned short *getBodyIndexData() const;
    bool isBodyIndexDataAvailable() const;

    const unsigned short *getIRData() const;
    bool isIRDataAvailable() const;

    /** Body Detection Parameters **/

    /**
     * @brief Any skeleton that is behind the closest skeleton by this offset is deleted. If it equals to 0 and calculation is not forced this removing is not done.
     * @param offset
     */
    void setClosestBodyOffset(const float &offset);
    void resetClosestBodyOffset();
    const float &getClosestBodyOffset() const;

    bool isClosestBodyCalculationForced() const;
    void setForceClosestBodyCalculation(bool forced);

    unsigned int getDesiredBodyCount() const;

    /**
     * @brief This is done after closest body calculation. The bodies closest to the center are kept
     * @param desiredBodyCount
     */
    void setDesiredBodyCount(unsigned int desiredBodyCount);

    const UINT64 &getClosestBodyID() const;

    float getMaxBodyDistance() const;

    /**
     * @brief Any skeleton that is beyond the distance is not processed.
     * @param distance
     */
    void setMaxBodyDistance(float distance);
    void resetMaxBodyDistance();

    /** IR Properties **/

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

    /** Depth Property Getters **/

    USHORT getDepthMinReliableDistance() const;
    USHORT getDepthMaxReliableDistance() const;

private:
    bool m_IsColorDataAvailable,// This is set to true when the Kinect color processing is done
         m_IsDepthDataAvailable,// This is set to true when the Kinect depth processing is done
         m_IsBodyIndexDataAvailable,// This is set to true when the Kinect body index processing is done
         m_IsIRDataAvailable,// This is set to true when the Kinect infrred processing is done
         m_CanTakeSnapshot,
         m_IsSensorClosed,
         m_IsForceClosestBodyCalculation;// This ensures that even when the visible and desired body count is the same closest body calculation is done

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

    unsigned int m_InitType;

    std::thread m_ThreadUpdate, m_ThreadScreenshot;
    std::recursive_mutex m_Mutex;

    /**
     * @brief Tracking ID for the closest skeleton on the Z-axis
     */
    UINT64 m_ClosestBodyID;

    /**
     * @brief Any skeleton that is behind the closest skeleton by this offset is deleted. If it equals to 0 this removing is not done.
     * If the visible body count equals to the desired body count and m_IsForceClosestBodyCalculation is true then this calclation is skipped
     */
    float m_ClosestBodyOffset;

    /**
     * @brief Any skeleton that is farther from the sensor on the Z-axis than m_MaxBodyDistance will not be processed. The default value is 0.
     */
    float m_MaxBodyDistance;

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

    DepthFrameInfo m_DepthFrameInfo;
    ColorFrameInfo m_ColorFrameInfo;
    BodyIndexFrameInfo m_BodyIndexFrameInfo;
    IRFrameInfo m_IRFrameInfo;
    BodyFrameInfo m_BodyFrameInfo;

    /**
     * @brief This map is filled in the beginning of each frame and freed at the end of each frame.
     * This represents the joints of the available skeletons in the current frame. This is not meant to be shared with outside of the class.
     */
    std::map<UINT64, Joint *> m_AllJoints;

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
    void processBody(const UINT64 &delta, const int &bodyCount, IBody **bodies);
    void processClosestBodyConstraint(std::array<IBody *, BODY_COUNT> &visibleBodies);
    void processDesiredBodyCount(std::array<IBody *, BODY_COUNT> &visibleBodies);

    /** Body Sort Functions **/

    bool sortBodyZDesc(IBody *bodyOne, IBody *bodyTwo) const;

    /**
     * @brief Sort the bodies from left to right on the X-axis. Player one is the left-most body.
     * @param bodyOne
     * @param bodyTwo
     * @return
     */
    bool sortBodyXAsc(IBody *bodyOne, IBody *bodyTwo) const;

    /**
     * @brief Uses the absolute value to put the ones that are closer to zero on the left.
     * So, visibleBodies.at(0) is the closest one to the and visibleBodies.at(visibleBodies.size() - 1) is the farthest from the center.
     * @param bodyOne
     * @param bodyTwo
     * @return
     */
    bool sortBodyCenter(IBody *bodyOne, IBody *bodyTwo) const;

    /**
     * @brief Sorts the bodies in a way that the index 0 becomes the index closest to the sensor and the center
     * @param bodyOne
     * @param bodyTwo
     * @return
     */
    bool sortBodyCenterAndZDesc(IBody *bodyOne, IBody *bodyTwo) const;

    /** Frame Update Functions **/

    HRESULT updateDepthFrameData();
    HRESULT updateColorFrameData();
    HRESULT updateBodyIndexFrameData();
    HRESULT updateBodyFrame();
    HRESULT updateIRFrameData();
};

#endif // KINECTHANDLER_H
