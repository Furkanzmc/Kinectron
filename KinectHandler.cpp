#include "KinectHandler.h"
#include <iostream>
#include <ctime>
#include <vector>
#include <algorithm>
#include <string>

KinectHandler::KinectHandler()
    : m_isColorDataAvailable(false)
    , m_isBackgroundRemovedDataAvailable(false)
    , m_isBackgroundRemovalEnabled(false)
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
    , m_OutputRGBX(nullptr)
    , m_BackgroundRGBX(nullptr)
    , m_ColorRGBX(nullptr)
    , m_InitType(FrameSourceTypes_None)
    , m_FrameArrivedHandle(NULL)
    , m_ClosestBodyID(0)
    , m_ClosestBodyOffset(-1.f)
    , m_DesiredBodyCount(BODY_COUNT)
{
    // create heap storage for composite image pixel data in RGBX format
    m_OutputRGBX = new RGBQUAD[COLOR_WIDTH * COLOR_HEIGHT];

    // create heap storage for background image pixel data in RGBX format
    m_BackgroundRGBX = new RGBQUAD[COLOR_WIDTH * COLOR_HEIGHT];

    // create heap storage for color pixel data in RGBX format
    m_ColorRGBX = new RGBQUAD[COLOR_WIDTH * COLOR_HEIGHT];

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

    if (m_OutputRGBX) {
        delete[] m_OutputRGBX;
        m_OutputRGBX = nullptr;
    }
    if (m_BackgroundRGBX) {
        delete[] m_BackgroundRGBX;
        m_BackgroundRGBX = nullptr;
    }
    if (m_ColorRGBX) {
        delete[] m_ColorRGBX;
        m_ColorRGBX = nullptr;
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
    return reinterpret_cast<unsigned char *>(m_ColorRGBX);;
}

const unsigned char *KinectHandler::getBackgroundRemovedData() const
{
    return reinterpret_cast<unsigned char *>(m_OutputRGBX);
}

bool KinectHandler::isColorDataAvailable() const
{
    return m_isColorDataAvailable;
}

bool KinectHandler::isBackgroundRemovedDataAvailable() const
{
    return m_isBackgroundRemovedDataAvailable && m_BackgroundRGBX != nullptr;
}

void KinectHandler::setBackgroundRemovalEnabled(const bool &isEnabled)
{
    m_isBackgroundRemovalEnabled = isEnabled;
}

bool KinectHandler::isBackgroundRemovalEnabled() const
{
    return m_isBackgroundRemovalEnabled;
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
            hr = updateBodyIndexFrameData(m_BodyIndexInfo, m_BodyIndexFrame);
            if (SUCCEEDED(hr)) {
                if (isBackgroundRemovalEnabled() && (m_ColorFrame && m_DepthFrame && m_BodyIndexFrame)) {
                    extractPlayerFromBackground();
                }
            }

            //Release the frame descriptions
            safeRelease(m_DepthInfo.depthFrameDescription);
            safeRelease(m_ColorFrameInfo.colorFrameDescription);
            safeRelease(m_BodyIndexInfo.bodyIndexFrameDescription);
        }
    }
}

void KinectHandler::extractPlayerFromBackground()
{
    std::lock_guard<std::recursive_mutex> lock(m_Mutex);
    if (!m_ColorFrame || !m_DepthFrame || !m_BodyIndexFrame) {
        return;
    }

    m_isBackgroundRemovedDataAvailable = false;
    const bool isDepthAvailable = m_DepthInfo.depthBuffer && m_DepthInfo.depthWidth == DEPTH_WIDTH && m_DepthInfo.depthHeight == DEPTH_HEIGHT;
    const bool isColorAvailable = m_ColorFrameInfo.colorBuffer && m_ColorFrameInfo.colorWidth == COLOR_WIDTH && m_ColorFrameInfo.colorHeight == COLOR_HEIGHT;
    const bool isBodyIndexAvailable = m_BodyIndexInfo.bodyIndexBuffer && m_BodyIndexInfo.bodyIndexWidth == DEPTH_WIDTH
                                      && m_BodyIndexInfo.bodyIndexHeight == DEPTH_HEIGHT;
    // Make sure we've received valid data
    if (m_CoordinateMapper && m_DepthCoordinates && m_OutputRGBX && isDepthAvailable && isColorAvailable && isBodyIndexAvailable) {
        HRESULT hr = m_CoordinateMapper->MapColorFrameToDepthSpace(m_DepthInfo.depthWidth * m_DepthInfo.depthHeight, m_DepthInfo.depthBuffer,
                     m_ColorFrameInfo.colorWidth * m_ColorFrameInfo.colorHeight, m_DepthCoordinates);

        if (SUCCEEDED(hr)) {
            // loop over output pixels
            for (int colorIndex = 0; colorIndex < (m_ColorFrameInfo.colorWidth * m_ColorFrameInfo.colorHeight); ++colorIndex) {
                //Default setting source to copy from the background pixel
                const RGBQUAD *pSrc = m_BackgroundRGBX + 0;
                DepthSpacePoint p = m_DepthCoordinates[colorIndex];

                //Values that are negative infinity means it is an invalid color to depth mapping so we skip processing for this pixel
                if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity()) {
                    const int depthX = static_cast<int>(p.X + 0.5f);
                    const int depthY = static_cast<int>(p.Y + 0.5f);

                    if ((depthX >= 0 && depthX < m_DepthInfo.depthWidth) && (depthY >= 0 && depthY < m_DepthInfo.depthHeight)) {
                        UINT8 player = m_BodyIndexInfo.bodyIndexBuffer[depthX + (depthY * DEPTH_WIDTH)];

                        //If we're tracking a player for the current pixel, draw from the color camera
                        if (player != 0xff) {
                            //Set source for copy to the color pixel
                            pSrc = m_ColorRGBX + colorIndex;
                        }
                    }
                }
                //Write output
                m_OutputRGBX[colorIndex] = *pSrc;
            }
            m_isBackgroundRemovedDataAvailable = true;
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
        hr = depthFrame->get_FrameDescription(&depthInfo.depthFrameDescription);
    }
    if (SUCCEEDED(hr)) {
        hr = depthInfo.depthFrameDescription->get_Width(&depthInfo.depthWidth);
    }
    if (SUCCEEDED(hr)) {
        hr = depthInfo.depthFrameDescription->get_Height(&depthInfo.depthHeight);
    }
    if (SUCCEEDED(hr)) {
        hr = depthFrame->AccessUnderlyingBuffer(&depthInfo.depthBufferSize, &depthInfo.depthBuffer);
    }
    return hr;
}

HRESULT KinectHandler::updateColorFrameData(ColorFrameInfo &colorFrameInfo, IColorFrame *colorFrame)
{
    std::lock_guard<std::recursive_mutex> lock(m_Mutex);
    if (colorFrame == nullptr) {
        return E_FAIL;
    }

    HRESULT hr = colorFrame->get_FrameDescription(&colorFrameInfo.colorFrameDescription);
    if (SUCCEEDED(hr)) {
        hr = colorFrameInfo.colorFrameDescription->get_Width(&colorFrameInfo.colorWidth);
    }
    if (SUCCEEDED(hr)) {
        hr = colorFrameInfo.colorFrameDescription->get_Height(&colorFrameInfo.colorHeight);
    }
    if (SUCCEEDED(hr)) {
        hr = colorFrame->get_RawColorImageFormat(&colorFrameInfo.imageFormat);
    }
    if (SUCCEEDED(hr)) {
        if (colorFrameInfo.imageFormat == ColorImageFormat_Bgra) {
            hr = colorFrame->AccessRawUnderlyingBuffer(&colorFrameInfo.colorBufferSize, reinterpret_cast<BYTE **>(&colorFrameInfo.colorBuffer));
        }
        else if (m_ColorRGBX) {
            colorFrameInfo.colorBuffer = m_ColorRGBX;
            colorFrameInfo.colorBufferSize = COLOR_WIDTH * COLOR_HEIGHT * sizeof(RGBQUAD);
            hr = colorFrame->CopyConvertedFrameDataToArray(colorFrameInfo.colorBufferSize, reinterpret_cast<BYTE *>(colorFrameInfo.colorBuffer)
                    , ColorImageFormat_Rgba);
            m_ColorRGBX = colorFrameInfo.colorBuffer;
            m_isColorDataAvailable = true;
            if (m_TakeScreenshotFunc && m_CanTakeSnapshot) {
                if (m_ThreadScreenshot.joinable()) {
                    m_ThreadScreenshot.join();
                }
                m_ThreadScreenshot = std::thread(m_TakeScreenshotFunc, reinterpret_cast<unsigned char *>(m_ColorRGBX), DATA_LENGTH, COLOR_WIDTH, COLOR_HEIGHT,
                                                 BITS_PER_PIXEL, m_SnapshotFilePath);
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

    HRESULT hr = bodyIndexFrame->get_FrameDescription(&bodyIndexInfo.bodyIndexFrameDescription);
    if (SUCCEEDED(hr)) {
        hr = bodyIndexInfo.bodyIndexFrameDescription->get_Width(&bodyIndexInfo.bodyIndexWidth);
    }
    if (SUCCEEDED(hr)) {
        hr = bodyIndexInfo.bodyIndexFrameDescription->get_Height(&bodyIndexInfo.bodyIndexHeight);
    }
    if (SUCCEEDED(hr)) {
        hr = bodyIndexFrame->AccessUnderlyingBuffer(&bodyIndexInfo.bodyIndexBufferSize, &bodyIndexInfo.bodyIndexBuffer);
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
