#include "KinectHandler.h"
#include <ctime>
#include <vector>
#include <algorithm>
#include <string>

KinectHandler::KinectHandler()
    : m_IsColorDataAvailable(false)
    , m_IsDepthDataAvailable(false)
    , m_IsBodyIndexDataAvailable(false)
    , m_IsIRDataAvailable(false)
    , m_CanTakeSnapshot(false)
    , m_IsSensorClosed(false)
    , m_IsForceClosestBodyCalculation(false)
    , m_SnapshotFilePath("")
    , m_Sensor(nullptr)
    , m_MultiSourceFrameReader(nullptr)
    , m_MultiSourceFrame(nullptr)
    , m_DepthFrame(nullptr)
    , m_ColorFrame(nullptr)
    , m_BodyIndexFrame(nullptr)
    , m_BodyFrame(nullptr)
    , m_IRFrame(nullptr)
    , m_CoordinateMapper(nullptr)
    , m_InitType(FrameSourceTypes_None)
    , m_ClosestBodyID(0)
    , m_ClosestBodyOffset(0)
    , m_MaxBodyDistance(0.f)
    , m_DesiredBodyCount(BODY_COUNT)
      // Frame Info
    , m_DepthFrameInfo()
    , m_ColorFrameInfo()
    , m_BodyIndexFrameInfo()
    , m_IRFrameInfo()
    , m_BodyFrameInfo()
{

}

KinectHandler::~KinectHandler()
{
    if (m_Sensor) {
        m_Sensor->Close();
        safeRelease(m_Sensor);
    }

    if (m_ThreadUpdate.joinable()) {
        m_ThreadUpdate.join();
    }

    if (m_ThreadScreenshot.joinable()) {
        m_ThreadScreenshot.join();
    }

    safeRelease(m_MultiSourceFrameReader);
    safeRelease(m_MultiSourceFrame);
    safeRelease(m_DepthFrame);
    safeRelease(m_ColorFrame);
    safeRelease(m_BodyIndexFrame);
    safeRelease(m_BodyFrame);
    safeRelease(m_IRFrame);
    safeRelease(m_CoordinateMapper);
}

HRESULT KinectHandler::initializeDefaultSensor(const unsigned int &initeType)
{
    HRESULT hr = E_FAIL;
    hr = GetDefaultKinectSensor(&m_Sensor);

    // Initialize the Kinect and get coordinate mapper and the frame reader
    if (SUCCEEDED(hr) && m_Sensor) {
        hr = m_Sensor->get_CoordinateMapper(&m_CoordinateMapper);

        if (SUCCEEDED(hr)) {
            hr = m_Sensor->Open();
        }
        if (SUCCEEDED(hr)) {
            hr = m_Sensor->OpenMultiSourceFrameReader(initeType, &m_MultiSourceFrameReader);
        }
        if (SUCCEEDED(hr)) {
            // Set this to false to not prevent the KinectHandler::updateSensor from running
            m_IsSensorClosed = false;
            m_InitType = initeType;
            m_ThreadUpdate = std::thread(&KinectHandler::updateSensor, this);
        }
    }

    return hr;
}

bool KinectHandler::isKinectAvailable()
{
    if (!m_Sensor) {
        return false;
    }

    BOOLEAN isAvaliable = 0;
    m_Sensor->get_IsAvailable(&isAvaliable);
    return isAvaliable == 1;
}

HRESULT KinectHandler::closeSensor()
{
    std::lock_guard<std::recursive_mutex> lock(m_Mutex);
    m_IsSensorClosed = true;
    return m_Sensor->Close();
}

void KinectHandler::reset()
{
    std::lock_guard<std::recursive_mutex> lock(m_Mutex);

    if (m_Sensor) {
        closeSensor();
        safeRelease(m_Sensor);
    }

    if (m_ThreadUpdate.joinable()) {
        m_ThreadUpdate.join();
    }

    if (m_ThreadScreenshot.joinable()) {
        m_ThreadScreenshot.join();
    }

    safeRelease(m_MultiSourceFrameReader);
    safeRelease(m_MultiSourceFrame);
    safeRelease(m_DepthFrame);
    safeRelease(m_ColorFrame);
    safeRelease(m_BodyIndexFrame);
    safeRelease(m_BodyFrame);
    safeRelease(m_IRFrame);
    safeRelease(m_CoordinateMapper);

    m_IsColorDataAvailable = false;
    m_IsDepthDataAvailable = false;
    m_IsBodyIndexDataAvailable = false;
    m_IsIRDataAvailable = false;
    m_CanTakeSnapshot = false;

    // Resets the frame infos
    m_DepthFrameInfo.reset();
    m_ColorFrameInfo.reset();
    m_BodyIndexFrameInfo.reset();
    m_IRFrameInfo.reset();
    m_BodyFrameInfo.reset();
}

void KinectHandler::takeSnapshot(const std::string &filePath)
{
    m_CanTakeSnapshot = true;
    m_SnapshotFilePath = filePath;
}

const Vector4 &KinectHandler::getFloorClipPlane()
{
    return m_BodyFrameInfo.floorClipPlane;
}

bool KinectHandler::isFloorVisible() const
{
    bool floor = false;
    if (m_BodyFrameInfo.floorClipPlane.x != 0 && m_BodyFrameInfo.floorClipPlane.y != 0 && m_BodyFrameInfo.floorClipPlane.z != 0
        && m_BodyFrameInfo.floorClipPlane.w != 0) {
        floor = true;
    }

    return floor;
}

double KinectHandler::getDistanceFromFloor(const CameraSpacePoint &jointPosition) const
{
    // If floor isn't visible, can't get the distance
    if (!isFloorVisible()) {
        return -1;
    }

    const double floorClipPlaneX = m_BodyFrameInfo.floorClipPlane.x;
    const double floorClipPlaneY = m_BodyFrameInfo.floorClipPlane.y;
    const double floorClipPlaneZ = m_BodyFrameInfo.floorClipPlane.z;
    const double floorClipPlaneW = m_BodyFrameInfo.floorClipPlane.w;

    const double distanceFromFloor = floorClipPlaneX * jointPosition.X + floorClipPlaneY * jointPosition.Y + floorClipPlaneZ * jointPosition.Z + floorClipPlaneW;
    return distanceFromFloor;
}

ICoordinateMapper *KinectHandler::getCoordinateMapper() const
{
    return m_CoordinateMapper;
}

PointF KinectHandler::mapBodyPointToScreenPoint(const CameraSpacePoint &bodyPoint)
{
    // Calculate the body's position on the screen
    PointF screenPos = {0, 0};
    ColorSpacePoint colorPoint = {0, 0};

    if (m_CoordinateMapper) {
        m_CoordinateMapper->MapCameraPointToColorSpace(bodyPoint, &colorPoint);
        screenPos.X = colorPoint.X;
        screenPos.Y = colorPoint.Y;
    }

    return screenPos;
}

unsigned int KinectHandler::getInitType() const
{
    return m_InitType;
}

const unsigned char *KinectHandler::getColorData() const
{
    return reinterpret_cast<unsigned char *>(m_ColorFrameInfo.bufferRGBX);
}

const RGBQUAD *KinectHandler::getColorDataRGB() const
{
    return m_ColorFrameInfo.bufferRGBX;
}

bool KinectHandler::isColorDataAvailable() const
{
    return m_IsColorDataAvailable;
}

const unsigned short *KinectHandler::getDepthData() const
{
    return reinterpret_cast<unsigned short *>(m_DepthFrameInfo.bufferRGB);
}

bool KinectHandler::isDepthDataAvailable() const
{
    return m_IsDepthDataAvailable;
}

const unsigned short *KinectHandler::getBodyIndexData() const
{
    return reinterpret_cast<unsigned short *>(m_BodyIndexFrameInfo.bufferRGB);
}

bool KinectHandler::isBodyIndexDataAvailable() const
{
    return m_IsBodyIndexDataAvailable;
}

const unsigned short *KinectHandler::getIRData() const
{
    return reinterpret_cast<unsigned short *>(m_IRFrameInfo.bufferRGB);
}

bool KinectHandler::isIRDataAvailable() const
{
    return m_IsIRDataAvailable;
}

void KinectHandler::updateSensor()
{
    std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
    const double frameRate = 1.0 / 30.0;
    while (m_IsSensorClosed == false) {
        std::lock_guard<std::recursive_mutex> lock(m_Mutex);
        // Kinect does not support more than 30 frames per second, so don't update more than that.
        now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(now - start).count() < frameRate || m_MultiSourceFrameReader == nullptr) {
            continue;
        }
        start = std::chrono::high_resolution_clock::now();

        m_IsColorDataAvailable = false;
        m_IsDepthDataAvailable = false;
        m_IsBodyIndexDataAvailable = false;
        m_IsIRDataAvailable = false;
        // Safe release the frames before we can use them again
        safeRelease(m_DepthFrame);
        safeRelease(m_ColorFrame);
        safeRelease(m_BodyIndexFrame);
        safeRelease(m_BodyFrame);
        safeRelease(m_IRFrame);
        safeRelease(m_MultiSourceFrame);

        HRESULT hr = m_MultiSourceFrameReader->AcquireLatestFrame(&m_MultiSourceFrame);

        /* If the multi source frame cannot get a frame, it will return FAIL in that case safe release the frame
         * to later use that to determine If that frame has been requested or not (e.g If m_DepthFrame is a null pointer, then that frame hasn't been requested.)
         */
        if (SUCCEEDED(hr)) {
            /** Initialize depth stream **/
            IDepthFrameReference *depthFrameReference = nullptr;
            hr = m_MultiSourceFrame->get_DepthFrameReference(&depthFrameReference);
            if (SUCCEEDED(hr)) {
                hr = depthFrameReference->AcquireFrame(&m_DepthFrame);
            }
            else {
                safeRelease(m_DepthFrame);
            }
            safeRelease(depthFrameReference);

            /** Initialize color stream **/
            IColorFrameReference *colorFrameReference = nullptr;
            hr = m_MultiSourceFrame->get_ColorFrameReference(&colorFrameReference);
            if (SUCCEEDED(hr)) {
                hr = colorFrameReference->AcquireFrame(&m_ColorFrame);
            }
            else {
                safeRelease(m_ColorFrame);
            }
            safeRelease(colorFrameReference);

            /** Initialize body index stream **/
            IBodyIndexFrameReference *bodyIndexFrameReference = nullptr;
            hr = m_MultiSourceFrame->get_BodyIndexFrameReference(&bodyIndexFrameReference);
            if (SUCCEEDED(hr)) {
                hr = bodyIndexFrameReference->AcquireFrame(&m_BodyIndexFrame);
            }
            else {
                safeRelease(m_BodyIndexFrame);
            }
            safeRelease(bodyIndexFrameReference);

            /** Initialize body data **/
            IBodyFrameReference *bodyFrameReference = nullptr;
            hr = m_MultiSourceFrame->get_BodyFrameReference(&bodyFrameReference);
            if (SUCCEEDED(hr)) {
                hr = bodyFrameReference->AcquireFrame(&m_BodyFrame);
            }
            else {
                safeRelease(m_BodyFrame);
            }
            safeRelease(bodyFrameReference);

            /** Initialize IR data **/
            IInfraredFrameReference *irFrameReference = nullptr;
            hr = m_MultiSourceFrame->get_InfraredFrameReference(&irFrameReference);
            if (SUCCEEDED(hr)) {
                hr = irFrameReference->AcquireFrame(&m_IRFrame);
            }
            else {
                safeRelease(m_IRFrame);
            }
            safeRelease(irFrameReference);

            updateBodyFrame();
            updateColorFrameData();
            updateDepthFrameData();
            updateBodyIndexFrameData();
            updateIRFrameData();

            // Release the frame descriptions
            safeRelease(m_DepthFrameInfo.frameDescription);
            safeRelease(m_ColorFrameInfo.frameDescription);
            safeRelease(m_BodyIndexFrameInfo.frameDescription);
            safeRelease(m_IRFrameInfo.frameDescription);
        }
    }
}

void KinectHandler::processBody(const UINT64 &delta, const int &bodyCount, IBody **bodies)
{
    std::lock_guard<std::recursive_mutex> lock(m_Mutex);

    HRESULT hr = E_FAIL;
    std::array<IBody *, BODY_COUNT> visibleBodies;
    std::fill(visibleBodies.begin(), visibleBodies.end(), nullptr);

    if (m_CoordinateMapper == nullptr) {
        return;
    }
    // Go through the bodies and only get the visible ones
    for (int i = 0; i < bodyCount; ++i) {
        IBody *body = bodies[i];
        if (body) {
            BOOLEAN isBodyracked = false;
            hr = body->get_IsTracked(&isBodyracked);

            if (SUCCEEDED(hr) && isBodyracked) {
                Joint joints[JointType_Count];
                hr = body->GetJoints(_countof(joints), joints);

                if (SUCCEEDED(hr)) {
                    if (m_MaxBodyDistance > 0.f) {
                        if (joints[JointType_SpineBase].Position.Z < m_MaxBodyDistance) {
                            visibleBodies[i] = body;
                        }
                    }
                    else {
                        visibleBodies[i] = body;
                    }
                }
            }
        }
    }

    processClosestBodyConstraint(visibleBodies);
    processDesiredBodyCount(visibleBodies);

    if (onProcessBody) {
        onProcessBody(visibleBodies, delta);
    }
}

void KinectHandler::processClosestBodyConstraint(std::array<IBody *, BODY_COUNT> &visibleBodies)
{
    // Get the closest body index
    std::sort(visibleBodies.begin(), visibleBodies.end(), std::bind(&KinectHandler::sortBodyZDesc, this, std::placeholders::_1, std::placeholders::_2));
    const unsigned int closestBodyIndex = 0;
    IBody *closestBody = visibleBodies[closestBodyIndex];
    if (closestBody) {
        closestBody->get_TrackingId(&m_ClosestBodyID);
    }

    // Do the calculation only if the visible body count is bigger than the desired body count
    if (m_ClosestBodyOffset > 0 && (m_IsForceClosestBodyCalculation || visibleBodies.size() > m_DesiredBodyCount)) {
        // Remove the indexes that are not within the range, then sort the bodies from left to right again
        if (closestBody) {
            for (unsigned int bodyIndex = 0; bodyIndex < visibleBodies.size(); bodyIndex++) {
                IBody *body = visibleBodies[bodyIndex];
                if (!body) {
                    continue;
                }

                Joint jointsOne[JointType_Count];
                Joint jointsClosest[JointType_Count];
                body->GetJoints(_countof(jointsOne), jointsOne);
                closestBody->GetJoints(_countof(jointsClosest), jointsClosest);

                const CameraSpacePoint spineMidPosOne = jointsOne[JointType_SpineMid].Position;
                const CameraSpacePoint spineMidPosClosest = jointsClosest[JointType_SpineMid].Position;
                const bool shouldRemove = spineMidPosOne.Z > spineMidPosClosest.Z + m_ClosestBodyOffset;
                if (shouldRemove) {
                    visibleBodies[bodyIndex] = nullptr;
                }
            }
        }
    }
}

void KinectHandler::processDesiredBodyCount(std::array<IBody *, BODY_COUNT> &visibleBodies)
{
    if (m_DesiredBodyCount == BODY_COUNT) {
        std::sort(visibleBodies.begin(), visibleBodies.end(), std::bind(&KinectHandler::sortBodyXAsc, this, std::placeholders::_1, std::placeholders::_2));
        return;
    }

    std::sort(visibleBodies.begin(), visibleBodies.end(), std::bind(&KinectHandler::sortBodyCenter, this, std::placeholders::_1, std::placeholders::_2));

    // Start from the desired body size, and set the rest to nullptr
    for (unsigned int bodyIndex = m_DesiredBodyCount; bodyIndex < visibleBodies.size(); bodyIndex++) {
        visibleBodies[bodyIndex] = nullptr;
    }

    std::sort(visibleBodies.begin(), visibleBodies.end(), std::bind(&KinectHandler::sortBodyXAsc, this, std::placeholders::_1, std::placeholders::_2));
}

bool KinectHandler::sortBodyZDesc(IBody *bodyOne, IBody *bodyTwo) const
{
    if (bodyOne == nullptr && bodyTwo != nullptr) {
        return false;
    }
    else if (bodyOne != nullptr && bodyTwo == nullptr) {
        return true;
    }
    else if (bodyOne == nullptr && bodyTwo == nullptr) {
        return false;
    }

    Joint jointsOne[JointType_Count];
    Joint jointsTwo[JointType_Count];
    bodyOne->GetJoints(_countof(jointsOne), jointsOne);
    bodyTwo->GetJoints(_countof(jointsTwo), jointsTwo);

    const CameraSpacePoint spineMidPosOne = jointsOne[JointType_Head].Position;
    const CameraSpacePoint spineMidPosTwo = jointsTwo[JointType_Head].Position;

    return spineMidPosOne.Z < spineMidPosTwo.Z;
}

bool KinectHandler::sortBodyXAsc(IBody *bodyOne, IBody *bodyTwo) const
{
    bool isOnLeft = false;
    if (bodyOne == nullptr && bodyTwo != nullptr) {
        return false;
    }
    else if (bodyOne != nullptr && bodyTwo == nullptr) {
        return true;
    }
    else if (bodyOne == nullptr && bodyTwo == nullptr) {
        return false;
    }

    Joint jointsOne[JointType_Count];
    Joint jointsTwo[JointType_Count];
    bodyOne->GetJoints(_countof(jointsOne), jointsOne);
    bodyTwo->GetJoints(_countof(jointsTwo), jointsTwo);

    const CameraSpacePoint spineMidPosOne = jointsOne[JointType_Head].Position;
    const CameraSpacePoint spineMidPosTwo = jointsTwo[JointType_Head].Position;
    if (spineMidPosOne.X < spineMidPosTwo.X) {
        isOnLeft = true;
    }

    return isOnLeft;
}

bool KinectHandler::sortBodyCenter(IBody *bodyOne, IBody *bodyTwo) const
{
    bool isOnLeft = false;
    if (bodyOne == nullptr && bodyTwo != nullptr) {
        return false;
    }
    else if (bodyOne != nullptr && bodyTwo == nullptr) {
        return true;
    }
    else if (bodyOne == nullptr && bodyTwo == nullptr) {
        return false;
    }

    Joint jointsOne[JointType_Count];
    Joint jointsTwo[JointType_Count];
    bodyOne->GetJoints(_countof(jointsOne), jointsOne);
    bodyTwo->GetJoints(_countof(jointsTwo), jointsTwo);

    const CameraSpacePoint spineMidPosOne = jointsOne[JointType_Head].Position;
    const CameraSpacePoint spineMidPosTwo = jointsTwo[JointType_Head].Position;
    if (std::abs(spineMidPosOne.X) < std::abs(spineMidPosTwo.X)) {
        isOnLeft = true;
    }

    return isOnLeft;
}

bool KinectHandler::sortBodyCenterAndZDesc(IBody *bodyOne, IBody *bodyTwo) const
{
    bool isOnLeft = false;
    if (bodyOne == nullptr && bodyTwo != nullptr) {
        return false;
    }
    else if (bodyOne != nullptr && bodyTwo == nullptr) {
        return true;
    }
    else if (bodyOne == nullptr && bodyTwo == nullptr) {
        return false;
    }

    Joint jointsOne[JointType_Count];
    Joint jointsTwo[JointType_Count];
    HRESULT hr = bodyOne->GetJoints(_countof(jointsOne), jointsOne);
    hr = bodyTwo->GetJoints(_countof(jointsTwo), jointsTwo);

    const CameraSpacePoint spineMidPosOne = jointsOne[JointType_Head].Position;
    const CameraSpacePoint spineMidPosTwo = jointsTwo[JointType_Head].Position;

    if (std::abs(spineMidPosOne.X) < std::abs(spineMidPosTwo.X) && std::abs(spineMidPosOne.Z) < std::abs(spineMidPosTwo.Z)) {
        isOnLeft = true;
    }

    return isOnLeft;
}

HRESULT KinectHandler::updateDepthFrameData()
{
    std::lock_guard<std::recursive_mutex> lock(m_Mutex);
    if (m_DepthFrame == nullptr) {
        return E_FAIL;
    }

    HRESULT hr = m_DepthFrame->get_RelativeTime(&m_DepthFrameInfo.time);
    if (SUCCEEDED(hr)) {
        hr = m_DepthFrame->get_FrameDescription(&m_DepthFrameInfo.frameDescription);
    }
    if (SUCCEEDED(hr)) {
        hr = m_DepthFrameInfo.frameDescription->get_Width(&m_DepthFrameInfo.width);
    }
    if (SUCCEEDED(hr)) {
        hr = m_DepthFrameInfo.frameDescription->get_Height(&m_DepthFrameInfo.height);
    }
    if (SUCCEEDED(hr)) {
        hr = m_DepthFrame->get_DepthMinReliableDistance(&m_DepthFrameInfo.minReliableDistance);
    }
    if (SUCCEEDED(hr)) {
        hr = m_DepthFrame->get_DepthMaxReliableDistance(&m_DepthFrameInfo.maxReliableDistance);
    }
    if (SUCCEEDED(hr)) {
        hr = m_DepthFrame->AccessUnderlyingBuffer(&m_DepthFrameInfo.bufferSize, &m_DepthFrameInfo.buffer);
    }

    // Process the depth image
    if (SUCCEEDED(hr) && m_DepthFrameInfo.buffer != nullptr && m_DepthFrameInfo.width == DEPTH_WIDTH && m_DepthFrameInfo.height == DEPTH_HEIGHT) {
        if (m_DepthFrameInfo.bufferRGB == nullptr) {
            m_DepthFrameInfo.bufferRGB = new RGBTRIPLE[DEPTH_WIDTH * DEPTH_HEIGHT];
        }

        RGBTRIPLE *rgb = m_DepthFrameInfo.bufferRGB;
        UINT16 *buffer = m_DepthFrameInfo.buffer;

        // End pixel is start + width * height - 1
        const UINT16 *bufferEnd = buffer + (m_DepthFrameInfo.width * m_DepthFrameInfo.height);

        while (buffer < bufferEnd) {
            const USHORT depth = *buffer;

            // To convert to a byte, we're discarding the most-significant
            // rather than least-significant bits.
            // We're preserving detail, although the intensity will "wrap".
            // Values outside the reliable depth range are mapped to 0 (black).

            // Note: Using conditionals in this loop could degrade performance.
            // Consider using a lookup table instead when writing production code.
            const BYTE intensity = static_cast<BYTE>((depth >= m_DepthFrameInfo.minReliableDistance)
                                   && (depth <= m_DepthFrameInfo.maxReliableDistance) ? (depth % 256) : 0);

            rgb->rgbtRed = intensity;
            rgb->rgbtGreen = intensity;
            rgb->rgbtBlue = intensity;

            ++rgb;
            ++buffer;
        }

        m_IsDepthDataAvailable = true;
    }

    return hr;
}

HRESULT KinectHandler::updateColorFrameData()
{
    std::lock_guard<std::recursive_mutex> lock(m_Mutex);
    if (m_ColorFrame == nullptr) {
        return E_FAIL;
    }

    HRESULT hr = m_ColorFrame->get_RelativeTime(&m_ColorFrameInfo.time);
    if (SUCCEEDED(hr)) {
        hr = m_ColorFrame->get_FrameDescription(&m_ColorFrameInfo.frameDescription);
    }
    if (SUCCEEDED(hr)) {
        hr = m_ColorFrameInfo.frameDescription->get_Width(&m_ColorFrameInfo.width);
    }
    if (SUCCEEDED(hr)) {
        hr = m_ColorFrameInfo.frameDescription->get_Height(&m_ColorFrameInfo.height);
    }
    if (SUCCEEDED(hr)) {
        hr = m_ColorFrame->get_RawColorImageFormat(&m_ColorFrameInfo.imageFormat);
    }
    if (SUCCEEDED(hr)) {
        if (m_ColorFrameInfo.imageFormat == ColorImageFormat_Bgra) {
            hr = m_ColorFrame->AccessRawUnderlyingBuffer(&m_ColorFrameInfo.bufferSize, reinterpret_cast<BYTE **>(&m_ColorFrameInfo.bufferRGBX));
        }
        else if (SUCCEEDED(hr)) {
            if (m_ColorFrameInfo.bufferRGBX == nullptr) {
                m_ColorFrameInfo.bufferRGBX = new RGBQUAD[COLOR_WIDTH * COLOR_HEIGHT];;
            }

            m_ColorFrameInfo.bufferSize = COLOR_WIDTH * COLOR_HEIGHT * sizeof(RGBQUAD);
            hr = m_ColorFrame->CopyConvertedFrameDataToArray(m_ColorFrameInfo.bufferSize, reinterpret_cast<BYTE *>(m_ColorFrameInfo.bufferRGBX), ColorImageFormat_Rgba);
            m_IsColorDataAvailable = true;
            if (onTakeScreenshot && m_CanTakeSnapshot) {
                if (m_ThreadScreenshot.joinable()) {
                    m_ThreadScreenshot.join();
                }
                m_ThreadScreenshot = std::thread(onTakeScreenshot, reinterpret_cast<unsigned char *>(m_ColorFrameInfo.bufferRGBX),
                                                 DATA_LENGTH_COLOR, COLOR_WIDTH, COLOR_HEIGHT, BITS_PER_PIXEL_COLOR, m_SnapshotFilePath);
                m_CanTakeSnapshot = false;
            }
        }
        else {
            hr = E_FAIL;
        }
    }
    return hr;
}

HRESULT KinectHandler::updateBodyIndexFrameData()
{
    std::lock_guard<std::recursive_mutex> lock(m_Mutex);
    if (m_BodyIndexFrame == nullptr) {
        return E_FAIL;
    }

    HRESULT hr = m_BodyIndexFrame->get_RelativeTime(&m_BodyIndexFrameInfo.time);
    if (SUCCEEDED(hr)) {
        hr = m_BodyIndexFrame->get_FrameDescription(&m_BodyIndexFrameInfo.frameDescription);
    }
    if (SUCCEEDED(hr)) {
        hr = m_BodyIndexFrameInfo.frameDescription->get_Width(&m_BodyIndexFrameInfo.width);
    }
    if (SUCCEEDED(hr)) {
        hr = m_BodyIndexFrameInfo.frameDescription->get_Height(&m_BodyIndexFrameInfo.height);
    }
    if (SUCCEEDED(hr)) {
        hr = m_BodyIndexFrame->AccessUnderlyingBuffer(&m_BodyIndexFrameInfo.bufferSize, &m_BodyIndexFrameInfo.buffer);
    }

    // Process body index
    if (SUCCEEDED(hr) && m_BodyIndexFrameInfo.buffer != nullptr && m_BodyIndexFrameInfo.width == BODY_INDEX_WIDTH
        && m_BodyIndexFrameInfo.height == BODY_INDEX_HEIGHT) {
        if (m_BodyIndexFrameInfo.bufferRGB == nullptr) {
            m_BodyIndexFrameInfo.bufferRGB = new RGBTRIPLE[BODY_INDEX_WIDTH * BODY_INDEX_HEIGHT];
        }

        RGBTRIPLE *rgb = m_BodyIndexFrameInfo.bufferRGB;
        UINT8 *buffer = m_BodyIndexFrameInfo.buffer;

        // End pixel is start + width * height - 1
        const UINT8 *bufferEnd = buffer + (m_BodyIndexFrameInfo.width * m_BodyIndexFrameInfo.height);

        while (buffer < bufferEnd) {
            const USHORT pixel = *buffer;

            rgb->rgbtRed = pixel < 6 ? 255 : 0;
            rgb->rgbtGreen = 0;
            rgb->rgbtBlue = 0;

            ++rgb;
            ++buffer;
        }

        m_IsBodyIndexDataAvailable = true;
    }

    return hr;
}

HRESULT KinectHandler::updateBodyFrame()
{
    std::lock_guard<std::recursive_mutex> lock(m_Mutex);
    if (m_BodyFrame == nullptr) {
        return E_FAIL;
    }

    HRESULT hr = m_BodyFrame->get_FloorClipPlane(&m_BodyFrameInfo.floorClipPlane);
    if (SUCCEEDED(hr)) {
        hr = m_BodyFrame->get_RelativeTime(&m_BodyFrameInfo.time);
        IBody *bodies[BODY_COUNT] = {nullptr};

        if (SUCCEEDED(hr)) {
            hr = m_BodyFrame->GetAndRefreshBodyData(_countof(bodies), bodies);
        }
        if (SUCCEEDED(hr)) {
            processBody(m_BodyFrameInfo.time, BODY_COUNT, bodies);
        }
    }

    return hr;
}

HRESULT KinectHandler::updateIRFrameData()
{
    std::lock_guard<std::recursive_mutex> lock(m_Mutex);
    if (m_IRFrame == nullptr) {
        return E_FAIL;
    }

    HRESULT hr = m_IRFrame->get_RelativeTime(&m_IRFrameInfo.time);
    if (SUCCEEDED(hr)) {
        hr = m_IRFrame->get_FrameDescription(&m_IRFrameInfo.frameDescription);
    }
    if (SUCCEEDED(hr)) {
        hr = m_IRFrameInfo.frameDescription->get_Width(&m_IRFrameInfo.width);
    }
    if (SUCCEEDED(hr)) {
        hr = m_IRFrameInfo.frameDescription->get_Height(&m_IRFrameInfo.height);
    }
    if (SUCCEEDED(hr)) {
        hr = m_IRFrame->AccessUnderlyingBuffer(&m_IRFrameInfo.bufferSize, &m_IRFrameInfo.buffer);
    }

    // Process IR frame
    if (SUCCEEDED(hr) && m_IRFrameInfo.buffer && m_IRFrameInfo.width == IR_WIDTH && m_IRFrameInfo.height == IR_HEIGHT) {
        if (m_IRFrameInfo.bufferRGB == nullptr) {
            m_IRFrameInfo.bufferRGB = new RGBTRIPLE[IR_WIDTH * IR_HEIGHT];
        }

        RGBTRIPLE *pDest = m_IRFrameInfo.bufferRGB;
        UINT16 *pBuffer = m_IRFrameInfo.buffer;

        // End pixel is start + width * height - 1
        const UINT16 *pBufferEnd = pBuffer + (IR_WIDTH * IR_HEIGHT);

        while (pBuffer < pBufferEnd) {
            /** Normalize the incoming infrared data (ushort) to a float ranging from [InfraredOutputValueMinimum, InfraredOutputValueMaximum] by
             * 1. Dividing the incoming value by the source maximum value
             **/
            float intensityRatio = static_cast<float>(*pBuffer) / m_IRFrameInfo.sourceValueMaximum;

            // 2. Dividing by the (average scene value * standard deviations)
            intensityRatio /= m_IRFrameInfo.sceneValueAverage * m_IRFrameInfo.sceneStandardDeviations;

            // 3. Limiting the value to InfraredOutputValueMaximum
            intensityRatio = min(m_IRFrameInfo.outputValueMaximum, intensityRatio);

            // 4. Limiting the lower value InfraredOutputValueMinimym
            intensityRatio = max(m_IRFrameInfo.outputValueMinimum, intensityRatio);

            // 5. Converting the normalized value to a byte and using the result as the RGB components required by the image
            const byte intensity = static_cast<byte>(intensityRatio * 255.0f);
            pDest->rgbtRed = intensity;
            pDest->rgbtGreen = intensity;
            pDest->rgbtBlue = intensity;

            ++pDest;
            ++pBuffer;
        }

        m_IsIRDataAvailable = true;
    }

    return hr;
}

void KinectHandler::setClosestBodyOffset(const float &offset)
{
    m_ClosestBodyOffset = max(0, offset);
}

void KinectHandler::resetClosestBodyOffset()
{
    m_ClosestBodyOffset = 0.f;
}

const float &KinectHandler::getClosestBodyOffset() const
{
    return m_ClosestBodyOffset;
}

bool KinectHandler::isClosestBodyCalculationForced() const
{
    return m_IsForceClosestBodyCalculation;
}

void KinectHandler::setForceClosestBodyCalculation(bool forced)
{
    m_IsForceClosestBodyCalculation = forced;
}

unsigned int KinectHandler::getDesiredBodyCount() const
{
    return m_DesiredBodyCount;
}

void KinectHandler::setDesiredBodyCount(unsigned int desiredBodyCount)
{
    m_DesiredBodyCount = desiredBodyCount > BODY_COUNT ? BODY_COUNT :
                         (desiredBodyCount == 0 ? 1 : desiredBodyCount);
}

const UINT64 &KinectHandler::getClosestBodyID() const
{
    return m_ClosestBodyID;
}

float KinectHandler::getMaxBodyDistance() const
{
    return m_MaxBodyDistance;
}

void KinectHandler::setMaxBodyDistance(float distance)
{
    m_MaxBodyDistance = max(0, distance);
}

void KinectHandler::resetMaxBodyDistance()
{
    m_MaxBodyDistance = 0.f;
}

void KinectHandler::setIRSourceValueMax(float maxVal)
{
    m_IRFrameInfo.sourceValueMaximum = maxVal;
}

float KinectHandler::getIRSourceValueMax() const
{
    return m_IRFrameInfo.sourceValueMaximum;
}

void KinectHandler::setIROutputValueMin(float minVal)
{
    m_IRFrameInfo.outputValueMinimum = minVal;
}

float KinectHandler::getIROutputValueMin() const
{
    return m_IRFrameInfo.outputValueMinimum;
}

void KinectHandler::setIROutputValueMax(float maxVal)
{
    m_IRFrameInfo.outputValueMaximum = maxVal;
}

float KinectHandler::getIROutputValueMax() const
{
    return m_IRFrameInfo.outputValueMaximum;
}

void KinectHandler::setIRSceneValueAvg(float avgVal)
{
    m_IRFrameInfo.sceneValueAverage = avgVal;
}

float KinectHandler::getIRSceneValueAvg() const
{
    return m_IRFrameInfo.sceneValueAverage;
}

void KinectHandler::setIRSceneStandartDeviations(float deviation)
{
    m_IRFrameInfo.sceneStandardDeviations = deviation;
}

float KinectHandler::getIRSceneStandartDeviations() const
{
    return m_IRFrameInfo.sceneStandardDeviations;
}

USHORT KinectHandler::getDepthMinReliableDistance() const
{
    return m_DepthFrameInfo.minReliableDistance;
}

USHORT KinectHandler::getDepthMaxReliableDistance() const
{
    return m_DepthFrameInfo.maxReliableDistance;
}
