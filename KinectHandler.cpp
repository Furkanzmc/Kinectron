#include "KinectHandler.h"
#include <iostream>
#include <ctime>
#include <vector>
#include <algorithm>
#include <string>

KinectHandler::KinectHandler()
    : m_isColorDataAvailable(false)
    , m_isDepthDataAvailable(false)
    , m_isBodyIndexDataAvailable(false)
    , m_CanTakeSnapshot(false)
    , m_IsSensorClosed(false)
    , m_SnapshotFilePath("")
    , m_Sensor(nullptr)
    , m_MultiSourceFrameReader(nullptr)
    , m_MultiSourceFrame(nullptr)
    , m_DepthFrame(nullptr)
    , m_ColorFrame(nullptr)
    , m_BodyIndexFrame(nullptr)
    , m_BodyFrame(nullptr)
    , m_CoordinateMapper(nullptr)
    , m_DepthCoordinates(nullptr)
    , m_InitType(FrameSourceTypes_None)
    , m_FrameArrivedHandle(NULL)
    , m_ClosestBodyID(0)
    , m_ClosestBodyOffset(-1.f)
    , m_DesiredBodyCount(BODY_COUNT)
    , m_DepthInfo()
    , m_ColorFrameInfo()
    , m_BodyIndexInfo()
{
    // create heap storage for the coorinate mapping from color to depth
    m_DepthCoordinates = new DepthSpacePoint[COLOR_WIDTH * COLOR_HEIGHT];
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

    if (m_DepthCoordinates) {
        delete[] m_DepthCoordinates;
        m_DepthCoordinates = nullptr;
    }

    safeRelease(m_MultiSourceFrameReader);
    safeRelease(m_MultiSourceFrame);
    safeRelease(m_DepthFrame);
    safeRelease(m_ColorFrame);
    safeRelease(m_BodyIndexFrame);
    safeRelease(m_BodyFrame);
}

HRESULT KinectHandler::initializeDefaultSensor(const unsigned int &initeType)
{
    HRESULT hr = E_FAIL;
    hr = GetDefaultKinectSensor(&m_Sensor);

    // Initialize the Kinect and get coordinate mapper and the frame reader
    if (SUCCEEDED(hr) && m_Sensor) {
        hr = m_Sensor->get_CoordinateMapper(&m_CoordinateMapper);

        hr = m_Sensor->Open();
        if (SUCCEEDED(hr)) {
            hr = m_Sensor->OpenMultiSourceFrameReader(initeType, &m_MultiSourceFrameReader);
        }
        if (SUCCEEDED(hr)) {
            hr = m_MultiSourceFrameReader->SubscribeMultiSourceFrameArrived(&m_FrameArrivedHandle);
            if (SUCCEEDED(hr)) {
                m_ThreadUpdate = std::thread(&KinectHandler::updateSensor, this);
            }
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

void KinectHandler::takeSanpshot(const std::string &filePath)
{
    m_CanTakeSnapshot = true;
    m_SnapshotFilePath = filePath;
}

const Vector4 &KinectHandler::getFloorClipPlane()
{
    return m_FloorClipPlane;
}

bool KinectHandler::isFloorVisible() const
{
    // smooth out the skeleton data
    bool floor = false;
    if (m_FloorClipPlane.x != 0 && m_FloorClipPlane.y != 0 && m_FloorClipPlane.z != 0 && m_FloorClipPlane.w != 0) {
        floor = true;
    }
    return floor;
}

double KinectHandler::getDistanceFromFloor(const CameraSpacePoint &jointPosition) const
{
    //If floor isn't visible, can't get the distance
    if (!isFloorVisible()) {
        return -1;
    }
    double distanceFromFloor = 0;
    const double floorClipPlaneX = m_FloorClipPlane.x;
    const double floorClipPlaneY = m_FloorClipPlane.y;
    const double floorClipPlaneZ = m_FloorClipPlane.z;
    const double floorClipPlaneW = m_FloorClipPlane.w;
    distanceFromFloor = floorClipPlaneX * jointPosition.X + floorClipPlaneY * jointPosition.Y + floorClipPlaneZ * jointPosition.Z + floorClipPlaneW;
    return distanceFromFloor;
}

ICoordinateMapper *KinectHandler::getCoordinateMapper() const
{
    return m_CoordinateMapper;
}

const unsigned char *KinectHandler::getColorData() const
{
    return reinterpret_cast<unsigned char *>(m_ColorFrameInfo.bufferRGBX);;
}

bool KinectHandler::isColorDataAvailable() const
{
    return m_isColorDataAvailable;
}

const unsigned short *KinectHandler::getDepthData() const
{
    return reinterpret_cast<unsigned short *>(m_DepthInfo.bufferRGB);
}

bool KinectHandler::isDepthDataAvailable() const
{
    return m_isDepthDataAvailable;
}

const unsigned short *KinectHandler::getBodyIndexData() const
{
    return reinterpret_cast<unsigned short *>(m_BodyIndexInfo.bufferRGB);
}

bool KinectHandler::isBodyIndexDataAvailable() const
{
    return m_isBodyIndexDataAvailable;
}

void KinectHandler::updateSensor()
{
    std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
    while (m_IsSensorClosed == false) {
        std::lock_guard<std::recursive_mutex> lock(m_Mutex);
        //Kinect does not support more than 30 frames per second, so don't update more than that.
        now = std::chrono::high_resolution_clock::now();
        const float frameRate = 1.f / 30.f;
        if (std::chrono::duration<float>(now - start).count() < frameRate || m_MultiSourceFrameReader == nullptr) {
            continue;
        }
        start = std::chrono::high_resolution_clock::now();

        m_isColorDataAvailable = false;
        m_isDepthDataAvailable = false;
        m_isBodyIndexDataAvailable = false;
        //Safe release the frames before we can use them again
        safeRelease(m_DepthFrame);
        safeRelease(m_ColorFrame);
        safeRelease(m_BodyIndexFrame);
        safeRelease(m_BodyFrame);
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

            updateBodyFrame(m_BodyFrame);
            updateColorFrameData(m_ColorFrameInfo, m_ColorFrame);
            updateDepthFrameData(m_DepthInfo, m_DepthFrame);
            updateBodyIndexFrameData(m_BodyIndexInfo, m_BodyIndexFrame);

            //Release the frame descriptions
            safeRelease(m_DepthInfo.frameDescription);
            safeRelease(m_ColorFrameInfo.frameDescription);
            safeRelease(m_BodyIndexInfo.frameDescription);
        }
    }
}

void KinectHandler::processBody(UINT64 delta, int bodyCount, IBody **bodies)
{
    std::lock_guard<std::recursive_mutex> lock(m_Mutex);
    HRESULT hr = E_FAIL;
    std::array<IBody *, BODY_COUNT> visibleBodies;
    std::fill(visibleBodies.begin(), visibleBodies.end(), nullptr);

    if (m_CoordinateMapper) {
        //Go through the bodies and only get the visible ones
        for (int i = 0; i < bodyCount; ++i) {
            IBody *body = bodies[i];
            if (body) {
                BOOLEAN isBodyracked = false;
                hr = body->get_IsTracked(&isBodyracked);
                if (SUCCEEDED(hr) && isBodyracked) {
                    visibleBodies[i] = body;
                }
            }
        }

        //Get the closest body index
        auto descSortZ = [](IBody * bodyOne, IBody * bodyTwo) {
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
            return spineMidPosOne.Z < spineMidPosTwo.Z;
        };
        std::sort(visibleBodies.begin(), visibleBodies.end(), descSortZ);

        const unsigned int closestBodyIndex = 0;
        IBody *closestBody = visibleBodies[closestBodyIndex];
        if (closestBody) {
            closestBody->get_TrackingId(&m_ClosestBodyID);
        }

        if (m_ClosestBodyOffset > 0) {
            //Remove the indexes that are not within the range, then sort the bodies from left to right again
            if (closestBody) {
                for (unsigned int bodyIndex = 0; bodyIndex < visibleBodies.size(); bodyIndex++) {
                    IBody *body = visibleBodies[bodyIndex];
                    if (!body) {
                        continue;
                    }

                    Joint jointsOne[JointType_Count];
                    Joint jointsClosest[JointType_Count];
                    HRESULT hr = body->GetJoints(_countof(jointsOne), jointsOne);
                    if (SUCCEEDED(hr)) {
                        hr = closestBody->GetJoints(_countof(jointsClosest), jointsClosest);
                    }

                    const CameraSpacePoint spineMidPosOne = jointsOne[JointType_SpineMid].Position;
                    const CameraSpacePoint spineMidPosClosest = jointsClosest[JointType_SpineMid].Position;
                    const bool shouldRemove = spineMidPosOne.Z > spineMidPosClosest.Z + m_ClosestBodyOffset;
                    if (shouldRemove) {
                        visibleBodies[bodyIndex] = nullptr;
                    }
                }
            }
        }

        processDesiredBodyCount(visibleBodies);

        if (m_ProcessBodyFunc) {
            m_ProcessBodyFunc(visibleBodies, delta);
        }
    }
}

void KinectHandler::processDesiredBodyCount(std::array<IBody *, BODY_COUNT> &visibleBodies)
{
    //Sort the bodies from left to right on the X-axis. Player one is the left-most body.
    auto ascSort = [](IBody * bodyOne, IBody * bodyTwo) {
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
        if (spineMidPosOne.X < spineMidPosTwo.X) {
            isOnLeft = true;
        }
        return isOnLeft;
    };

    /* Use the absolute value to put the ones that are closer to zero on the left.
     * So, visibleBodies.at(0) is the closest one to the and visibleBodies.at(visibleBodies.size() - 1) is the farthest from the center.
     */
    auto centerSort = [](IBody * bodyOne, IBody * bodyTwo) {
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
        if (std::abs(spineMidPosOne.X) < std::abs(spineMidPosTwo.X)) {
            isOnLeft = true;
        }
        return isOnLeft;
    };

    if (m_DesiredBodyCount == BODY_COUNT) {
        std::sort(visibleBodies.begin(), visibleBodies.end(), ascSort);
        return;
    }

    std::sort(visibleBodies.begin(), visibleBodies.end(), centerSort);

    //Start from the desired body size, and set the rest to nullptr
    for (unsigned int bodyIndex = m_DesiredBodyCount; bodyIndex < visibleBodies.size(); bodyIndex++) {
        visibleBodies[bodyIndex] = nullptr;
    }

    std::sort(visibleBodies.begin(), visibleBodies.end(), ascSort);
}

HRESULT KinectHandler::updateDepthFrameData(DepthFrameInfo &depthInfo, IDepthFrame *depthFrame)
{
    std::lock_guard<std::recursive_mutex> lock(m_Mutex);
    if (depthFrame == nullptr) {
        return E_FAIL;
    }

    HRESULT hr = depthFrame->get_RelativeTime(&depthInfo.depthTime);
    if (SUCCEEDED(hr)) {
        hr = depthFrame->get_FrameDescription(&depthInfo.frameDescription);
    }
    if (SUCCEEDED(hr)) {
        hr = depthInfo.frameDescription->get_Width(&depthInfo.width);
    }
    if (SUCCEEDED(hr)) {
        hr = depthInfo.frameDescription->get_Height(&depthInfo.height);
    }
    if (SUCCEEDED(hr)) {
        hr = depthFrame->get_DepthMinReliableDistance(&depthInfo.minReliableDistance);
    }
    if (SUCCEEDED(hr)) {
        hr = depthFrame->get_DepthMaxReliableDistance(&depthInfo.maxReliableDistance);
    }
    if (SUCCEEDED(hr)) {
        hr = depthFrame->AccessUnderlyingBuffer(&depthInfo.bufferSize, &depthInfo.buffer);
    }

    //Process the depth image
    if (SUCCEEDED(hr) && depthInfo.buffer != nullptr && depthInfo.width == DEPTH_WIDTH && depthInfo.height == DEPTH_HEIGHT) {
        if (depthInfo.bufferRGB == nullptr) {
            depthInfo.bufferRGB = new RGBTRIPLE[DEPTH_WIDTH * DEPTH_HEIGHT];
        }

        RGBTRIPLE *rgb = depthInfo.bufferRGB;
        UINT16 *buffer = depthInfo.buffer;

        //End pixel is start + width * height - 1
        const UINT16 *bufferEnd = buffer + (depthInfo.width * depthInfo.height);

        while (buffer < bufferEnd) {
            const USHORT depth = *buffer;

            //To convert to a byte, we're discarding the most-significant
            //rather than least-significant bits.
            //We're preserving detail, although the intensity will "wrap".
            //Values outside the reliable depth range are mapped to 0 (black).

            //Note: Using conditionals in this loop could degrade performance.
            //Consider using a lookup table instead when writing production code.
            const BYTE intensity = static_cast<BYTE>((depth >= depthInfo.minReliableDistance) && (depth <= depthInfo.maxReliableDistance) ? (depth % 256) : 0);

            rgb->rgbtRed = intensity;
            rgb->rgbtGreen = intensity;
            rgb->rgbtBlue = intensity;

            ++rgb;
            ++buffer;
        }

        m_isDepthDataAvailable = true;
    }

    return hr;
}

HRESULT KinectHandler::updateColorFrameData(ColorFrameInfo &colorFrameInfo, IColorFrame *colorFrame)
{
    std::lock_guard<std::recursive_mutex> lock(m_Mutex);
    if (colorFrame == nullptr) {
        return E_FAIL;
    }

    HRESULT hr = colorFrame->get_FrameDescription(&colorFrameInfo.frameDescription);
    if (SUCCEEDED(hr)) {
        hr = colorFrameInfo.frameDescription->get_Width(&colorFrameInfo.width);
    }
    if (SUCCEEDED(hr)) {
        hr = colorFrameInfo.frameDescription->get_Height(&colorFrameInfo.height);
    }
    if (SUCCEEDED(hr)) {
        hr = colorFrame->get_RawColorImageFormat(&colorFrameInfo.imageFormat);
    }
    if (SUCCEEDED(hr)) {
        if (colorFrameInfo.imageFormat == ColorImageFormat_Bgra) {
            hr = colorFrame->AccessRawUnderlyingBuffer(&colorFrameInfo.bufferSize, reinterpret_cast<BYTE **>(&colorFrameInfo.bufferRGBX));
        }
        else if (SUCCEEDED(hr)) {
            if (colorFrameInfo.bufferRGBX == nullptr) {
                colorFrameInfo.bufferRGBX = new RGBQUAD[COLOR_WIDTH * COLOR_HEIGHT];;
            }

            colorFrameInfo.bufferSize = COLOR_WIDTH * COLOR_HEIGHT * sizeof(RGBQUAD);
            hr = colorFrame->CopyConvertedFrameDataToArray(colorFrameInfo.bufferSize, reinterpret_cast<BYTE *>(colorFrameInfo.bufferRGBX), ColorImageFormat_Rgba);
            m_isColorDataAvailable = true;
            if (m_TakeScreenshotFunc && m_CanTakeSnapshot) {
                if (m_ThreadScreenshot.joinable()) {
                    m_ThreadScreenshot.join();
                }
                m_ThreadScreenshot = std::thread(m_TakeScreenshotFunc, reinterpret_cast<unsigned char *>(colorFrameInfo.bufferRGBX),
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

HRESULT KinectHandler::updateBodyIndexFrameData(BodyIndexInfo &bodyIndexInfo, IBodyIndexFrame *bodyIndexFrame)
{
    std::lock_guard<std::recursive_mutex> lock(m_Mutex);
    if (bodyIndexFrame == nullptr) {
        return E_FAIL;
    }

    HRESULT hr = bodyIndexFrame->get_RelativeTime(&bodyIndexInfo.bodyIndexTime);
    if (SUCCEEDED(hr)) {
        hr = bodyIndexFrame->get_FrameDescription(&bodyIndexInfo.frameDescription);
    }
    if (SUCCEEDED(hr)) {
        hr = bodyIndexInfo.frameDescription->get_Width(&bodyIndexInfo.width);
    }
    if (SUCCEEDED(hr)) {
        hr = bodyIndexInfo.frameDescription->get_Height(&bodyIndexInfo.height);
    }
    if (SUCCEEDED(hr)) {
        hr = bodyIndexFrame->AccessUnderlyingBuffer(&bodyIndexInfo.bufferSize, &bodyIndexInfo.buffer);
    }

    //Process body index
    if (SUCCEEDED(hr) && bodyIndexInfo.buffer != nullptr && bodyIndexInfo.width == BODY_INDEX_WIDTH && bodyIndexInfo.height == BODY_INDEX_HEIGHT) {
        if (bodyIndexInfo.bufferRGB == nullptr) {
            bodyIndexInfo.bufferRGB = new RGBTRIPLE[BODY_INDEX_WIDTH * BODY_INDEX_HEIGHT];
        }

        RGBTRIPLE *rgb = bodyIndexInfo.bufferRGB;
        UINT8 *buffer = bodyIndexInfo.buffer;

        //End pixel is start + width * height - 1
        const UINT8 *bufferEnd = buffer + (bodyIndexInfo.width * bodyIndexInfo.height);

        while (buffer < bufferEnd) {
            const USHORT pixel = *buffer;

            rgb->rgbtRed = pixel < 6 ? 255 : 0;
            rgb->rgbtGreen = 0;
            rgb->rgbtBlue = 0;

            ++rgb;
            ++buffer;
        }

        m_isBodyIndexDataAvailable = true;
    }

    return hr;
}

HRESULT KinectHandler::updateBodyFrame(IBodyFrame *bodyFrame)
{
    std::lock_guard<std::recursive_mutex> lock(m_Mutex);
    if (bodyFrame == nullptr) {
        return E_FAIL;
    }

    HRESULT hr = bodyFrame->get_FloorClipPlane(&m_FloorClipPlane);
    if (SUCCEEDED(hr)) {
        INT64 time = 0;
        hr = m_BodyFrame->get_RelativeTime(&time);
        IBody *bodies[BODY_COUNT] = {nullptr};

        if (SUCCEEDED(hr)) {
            hr = m_BodyFrame->GetAndRefreshBodyData(_countof(bodies), bodies);
        }
        if (SUCCEEDED(hr)) {
            processBody(time, BODY_COUNT, bodies);
        }
    }
    return hr;
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

const UINT64 &KinectHandler::getClosestBodyID() const
{
    return m_ClosestBodyID;
}

void KinectHandler::setClosestBodyOffset(const float &offset)
{
    m_ClosestBodyOffset = offset;
}

const float &KinectHandler::getClosestBodyOffset() const
{
    return m_ClosestBodyOffset;
}

unsigned int KinectHandler::getDesiredBodyCount() const
{
    return m_DesiredBodyCount;
}

void KinectHandler::setDesiredBodyCount(unsigned int desiredBodyCount)
{
    m_DesiredBodyCount = desiredBodyCount > 6 ? 6 :
                         (desiredBodyCount == 0 ? 1 : desiredBodyCount);
}
