#include "Kinectron/GePoTool.h"
// Local
#include "Kinectron/KinectHandler.h"
// STD
#include <string>

GePoTool::GePoTool(const unsigned int &sensorIniteType)
    : m_KinectHandler(new KinectHandler())
    , m_SensorTime(0)
{
    std::fill(m_Bodies.begin(), m_Bodies.end(), nullptr);

    m_KinectHandler->initializeDefaultSensor(sensorIniteType);
    m_KinectHandler->onProcessBody = std::bind(&GePoTool::processBody, this, std::placeholders::_1, std::placeholders::_2);
}

GePoTool::~GePoTool()
{
    if (m_KinectHandler) {
        m_KinectHandler->closeSensor();
        delete m_KinectHandler;
        m_KinectHandler = nullptr;
    }
}

void GePoTool::processBody(const std::array<IBody *, BODY_COUNT> &bodyArray, UINT64 sensorTime)
{
    m_SensorTime = sensorTime;
    processBodyNotificitons(bodyArray);
    // Copy the new skeletons
    m_Bodies = bodyArray;
}

void GePoTool::processBodyNotificitons(const std::array<IBody *, BODY_COUNT> &bodyArray)
{
    auto countVisible = [](IBody * b) {
        return b != nullptr;
    };
    const std::size_t previousBodyCount = std::count_if(m_Bodies.begin(), m_Bodies.end(), countVisible);
    const std::size_t newBodyCount = std::count_if(bodyArray.begin(), bodyArray.end(), countVisible);

    if (newBodyCount == 0) {
        if (onBodyLost) {
            onBodyLost(PLAYER::ONE);
            onBodyLost(PLAYER::TWO);
            onBodyLost(PLAYER::THREE);
            onBodyLost(PLAYER::FOUR);
            onBodyLost(PLAYER::FIVE);
            onBodyLost(PLAYER::SIX);
        }
        return;
    }
    // If the m_Bodies is empty, copy the new vector and call the body found functions
    else if (previousBodyCount == 0) {
        for (unsigned int i = 0; i < newBodyCount; i++) {
            if (onBodyFound) {
                onBodyFound(i);
            }
        }
        return;
    }
    // If there are already items, and new bodyCount is bigger than the m_Bodies then new players joined.
    else if (newBodyCount > previousBodyCount) {
        for (std::size_t i = previousBodyCount - 1; i < newBodyCount; i++) {
            if (onBodyFound) {
                onBodyFound(i);
            }
        }
    }
    /* If the new body vector is smaller than the old one, then players left the battlefield. Figure out who the traitor is!!
     * If a body that is in the old one also exists in the new one, than he's OK. But if he does not... INFIDEL!!*/
    else if (newBodyCount < previousBodyCount) {
        for (BodyIndex oldBodyIndex = 0; oldBodyIndex < m_Bodies.size(); oldBodyIndex++) {
            if (m_Bodies.at(oldBodyIndex) == nullptr) {
                continue;
            }

            bool isBodyAlreadyExist = false;
            for (unsigned int newBodyIndex = 0; newBodyIndex < bodyArray.size(); newBodyIndex++) {
                if (bodyArray.at(newBodyIndex) == nullptr) {
                    continue;
                }

                if (getTrackingID(m_Bodies.at(oldBodyIndex)) == getTrackingID(bodyArray.at(newBodyIndex))) {
                    isBodyAlreadyExist = true;
                    // Yeap, he's ok.
                    break;
                }
            }
            // If the body does not exist, it can only mean one thing... INFIDEL!!!
            if (isBodyAlreadyExist == false) {
                if (onBodyLost) {
                    onBodyLost(oldBodyIndex);
                }
            }
        }
    }
}

UID GePoTool::getUID()
{
    if (m_UIDs.size() == 0) {
        m_UIDs.push_back(1);
        return 1;
    }

    const unsigned int newID = m_UIDs.at(m_UIDs.size() - 1) + 1;
    m_UIDs.push_back(newID);
    return newID;
}

DGestureBase *GePoTool::getDetector(const UID &detectorID)
{
    for (auto detector : m_Detectors) {
        if (detector->getUniqueID() == detectorID) {
            return detector;
        }
    }

    return nullptr;
}


bool GePoTool::removeDetector(DGestureBase *detector)
{
    if (detector == nullptr) {
        return false;
    }

    std::remove_if(m_Detectors.begin(), m_Detectors.end(), [detector](const DGestureBase * d) {
        return d->getUniqueID() == detector->getUniqueID();
    });

    delete detector;
    detector = nullptr;
    return true;
}

KinectHandler &GePoTool::getKinectHandler()
{
    return *m_KinectHandler;
}

IBody *GePoTool::getBody(const BodyIndex &player) const
{
    if (m_Bodies.size() <= player || m_Bodies.size() == 0) {
        return nullptr;
    }

    return m_Bodies.at(player);
}

IBody *GePoTool::getBodyWithTrackingID(const UINT64 &trackingID)
{
    if (m_Bodies.size() == 0) {
        return nullptr;
    }

    for (IBody *body : m_Bodies) {
        if (getTrackingID(body) == trackingID) {
            return body;
        }
    }

    return nullptr;
}

int GePoTool::getBodyIndexWithTrackingID(const UINT64 &trackingID)
{
    if (m_Bodies.size() == 0) {
        return -1;
    }

    int index = -1;
    int currentIndex = 0;
    for (IBody *body : m_Bodies) {
        if (getTrackingID(body) == trackingID) {
            index = currentIndex;
            break;
        }
        currentIndex++;
    }

    return index;
}

void GePoTool::resetBodies()
{
    std::fill(m_Bodies.begin(), m_Bodies.end(), nullptr);
}

UINT64 GePoTool::getTrackingID(IBody *body) const
{
    if (!body) {
        return 0;
    }

    UINT64 trackingID = 0;
    HRESULT hr = body->get_TrackingId(&trackingID);
    if (SUCCEEDED(hr)) {
        return trackingID;
    }

    return 0;
}

std::pair<HandState, HandState> GePoTool::getHandStates(IBody *body) const
{
    if (!body) {
        return std::make_pair(HandState_Unknown, HandState_Unknown);
    }

    if (isBodyTracked(body) == false) {
        return std::make_pair(HandState_Unknown, HandState_Unknown);
    }

    HandState leftHandState = HandState_Unknown;
    HandState rightHandState = HandState_Unknown;

    body->get_HandLeftState(&leftHandState);
    body->get_HandRightState(&rightHandState);

    return std::make_pair(leftHandState, rightHandState);
}

PointF GePoTool::getLeanAmount(IBody *body) const
{
    if (!body || isBodyTracked(body) == false) {
        PointF invalidLean = {0, 0};
        return invalidLean;
    }

    PointF leanAmount = {0, 0};
    body->get_Lean(&leanAmount);

    return leanAmount;
}

float GePoTool::getAngleBetweenHands(IBody *body) const
{
    if (!body || isBodyTracked(body) == false) {
        return static_cast<float>(INVALID_ANGLE);
    }

    Joint joints[JointType_Count];
    body->GetJoints(_countof(joints), joints);

    const CameraSpacePoint leftHand = joints[JointType_HandLeft].Position;
    const CameraSpacePoint rightHand = joints[JointType_HandRight].Position;

    // x -> Distance on x axis, y -> distance on y axis, h -> hypotenuse
    float x = 0, y = 0, h = 0;
    if (leftHand.X < 0 && rightHand.X < 0 || leftHand.X > 0 && rightHand.X > 0) {
        x = std::abs(leftHand.X - rightHand.X);
    }
    else if (leftHand.X < 0 && rightHand.X > 0) {
        x = std::abs(leftHand.X) + std::abs(rightHand.X);
    }

    y = std::abs(leftHand.Y - rightHand.Y);
    h = std::sqrtf(std::powf(leftHand.X - rightHand.X, 2) + std::powf(leftHand.Y - rightHand.Y, 2));
    const float radian = std::asinf(y / h);
    float angle = toDegree(radian);// Convert to degrees
    if (leftHand.Y < rightHand.Y) {
        angle *= -1;
    }

    return angle;
}

std::vector<UID> GePoTool::pollGestures(const BodyIndex &bodyIndex, const float &delta)
{
    std::vector<UID> detectedGestures;
    IBody *body = getBody(bodyIndex);
    if (!body) {
        return detectedGestures;
    }

    if (isBodyRestricted(body)) {
        return detectedGestures;
    }

    for (DGestureBase *detector : m_Detectors) {
        if (detector) {
            // If the detector already is set to a body index, and the given body inex does not eqaul to it skip the detector.
            if (detector->getBodyIndex() > DGestureBase::INVALID_BODY_INDEX && detector->getBodyIndex() != bodyIndex) {
                continue;
            }

            const UID uid = detector->detect(body, delta);
            if (uid == detector->getID()) {
                if (detector->onDetected) {
                    detector->onDetected(detector->getBodyIndex());
                }

                detectedGestures.push_back(uid);
            }
        }
    }
    if (detectedGestures.size() > 1) {
        // Sort in ascending order
        std::sort(detectedGestures.begin(), detectedGestures.end());
    }

    return detectedGestures;
}

UID GePoTool::determinePlayerPosture(const BodyIndex &bodyIndex, const float &delta)
{
    IBody *body = getBody(bodyIndex);
    if (!body) {
        return GePoTool::INVALID_UID;
    }

    if (isBodyRestricted(body)) {
        return GePoTool::INVALID_UID;
    }

    UID detectedGesture = GePoTool::INVALID_UID;
    for (DGestureBase *detector : m_Detectors) {
        if (detector) {
            // If the detector already is set to a body index, and the given body inex does not eqaul to it skip the detector.
            if (detector->getBodyIndex() > DGestureBase::INVALID_BODY_INDEX && detector->getBodyIndex() != bodyIndex) {
                continue;
            }

            const UID uid = detector->detect(body, delta);
            if (uid == detector->getID()) {
                if (detector->onDetected) {
                    detector->onDetected(detector->getBodyIndex());
                }

                detectedGesture = uid;
                break;
            }
        }
    }

    return detectedGesture;
}

std::size_t GePoTool::getDetectedBodyCount() const
{
    auto countVisible = [](IBody * b) {
        return b != nullptr;
    };

    return std::count_if(m_Bodies.begin(), m_Bodies.end(), countVisible);
}

GePoTool::BodyRect GePoTool::getBodyRect(IBody *body) const
{
    if (!body) {
        return BodyRect();
    }

    if (isBodyRestricted(body)) {
        return BodyRect();
    }

    const int headIndex = 0,
              handRightIndex = 1,
              handLeftIndex = 2,
              footRightIndex = 3,
              footLeftIndex = 4,
              shoulderRightIndex = 5,
              shoulderLeftIndex = 6;

    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        ColorSpacePoint colorPoints[7] = {0};
        CameraSpacePoint cameraPoints[7] = {0};

        cameraPoints[headIndex] = joints[JointType_Head].Position;
        cameraPoints[handRightIndex] = joints[JointType_HandRight].Position;
        cameraPoints[handLeftIndex] = joints[JointType_HandLeft].Position;
        cameraPoints[footRightIndex] = joints[JointType_FootRight].Position;
        cameraPoints[footLeftIndex] = joints[JointType_FootLeft].Position;
        cameraPoints[shoulderRightIndex] = joints[JointType_ShoulderRight].Position;
        cameraPoints[shoulderLeftIndex] = joints[JointType_ShoulderLeft].Position;

        // This is mapped for 1920x1080 resolution
        hr = m_KinectHandler->getCoordinateMapper()->MapCameraPointsToColorSpace(_countof(cameraPoints), cameraPoints, _countof(colorPoints), colorPoints);
        if (SUCCEEDED(hr)) {
            const float shoulderLenght = std::abs(colorPoints[shoulderLeftIndex].X - colorPoints[shoulderRightIndex].X) * 1.2f;

            BodyRect rect;
            rect.x = min(colorPoints[handLeftIndex].X, colorPoints[footLeftIndex].X);
            rect.y = min(min(colorPoints[headIndex].Y, colorPoints[handRightIndex].Y), colorPoints[handLeftIndex].Y) - shoulderLenght * .6f;

            const float xToRightHandLength = std::abs(rect.x - (colorPoints[handRightIndex].X));
            const float xToRightFootLength = std::abs(rect.x - (colorPoints[footRightIndex].X));
            const float maxWidth = max(shoulderLenght, max(xToRightHandLength, xToRightFootLength));

            const float yToRightFootLength = std::abs(rect.y - (colorPoints[footRightIndex].Y));
            const float yToLeftFootLength = std::abs(rect.y - (colorPoints[footLeftIndex].Y));
            const float maxHeight = max(yToRightFootLength, yToLeftFootLength);

            rect.width = maxWidth;
            rect.height = maxHeight;

            // Contain the rect within the color size
            if (rect.y < 0) {
                rect.y = 1;
            }
            else if (rect.y > KinectHandler::COLOR_HEIGHT) {
                rect.y = KinectHandler::COLOR_WIDTH;
            }

            if (rect.x > KinectHandler::COLOR_WIDTH) {
                rect.x = KinectHandler::COLOR_WIDTH;
            }
            else if (rect.x < 0) {
                rect.x = 1;
            }

            return rect;
        }
    }

    return BodyRect();
}

GePoTool::BodyRect GePoTool::getHeadRect(IBody *body) const
{
    if (!body) {
        return BodyRect();
    }

    if (isBodyRestricted(body)) {
        return BodyRect();
    }

    const unsigned int headIndex = 0, shoulderRightIndex = 1, shoulderLeftIndex = 2;
    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        if (joints[JointType_Head].TrackingState != TrackingState_Tracked) {
            return BodyRect();
        }

        std::array<PointF, 3> colorPoints = {0, 0, 0};
        std::array<CameraSpacePoint, 3> cameraPoints = {0, 0, 0};
        // This is mapped for 1920x1080 resolution
        cameraPoints[headIndex] = joints[JointType_Head].Position;
        cameraPoints[shoulderRightIndex] = joints[JointType_ShoulderRight].Position;
        cameraPoints[shoulderLeftIndex] = joints[JointType_ShoulderLeft].Position;

        colorPoints[headIndex] = m_KinectHandler->mapBodyPointToScreenPoint(cameraPoints[headIndex]);
        colorPoints[shoulderRightIndex] = m_KinectHandler->mapBodyPointToScreenPoint(cameraPoints[shoulderRightIndex]);
        colorPoints[shoulderLeftIndex] = m_KinectHandler->mapBodyPointToScreenPoint(cameraPoints[shoulderLeftIndex]);

        const float shoulderLenght = std::abs(colorPoints[shoulderLeftIndex].X - colorPoints[shoulderRightIndex].X) * 1.1f;

        BodyRect rect;
        rect.x = colorPoints[shoulderLeftIndex].X - shoulderLenght * 0.1f;
        rect.y = colorPoints[headIndex].Y - shoulderLenght / 2;
        if (rect.y < 0) {
            rect.y = 1;
        }
        else if (rect.y > KinectHandler::COLOR_HEIGHT) {
            rect.y = KinectHandler::COLOR_WIDTH;
        }

        if (rect.x > KinectHandler::COLOR_WIDTH) {
            rect.x = KinectHandler::COLOR_WIDTH;
        }
        else if (rect.x < 0) {
            rect.x = 1;
        }
        rect.width = shoulderLenght;
        rect.height = shoulderLenght;

        return rect;
    }

    return BodyRect();
}

float GePoTool::toDegree(const float &radian) const
{
    return (radian * 180.f) / 3.14158265f;
}

float GePoTool::toRadian(const float &degree) const
{
    return (degree * 3.14159265f) / 180.f;
}

Vector4 GePoTool::getJointOrientation(IBody *body, JointType joint) const
{
    if (!body) {
        return Vector4();
    }

    JointOrientation joints[JointType_Count];
    HRESULT hr = body->GetJointOrientations(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        return joints[joint].Orientation;
    }

    return Vector4();
}

HRESULT GePoTool::getJointOrientations(IBody *body, JointOrientation *orientations) const
{
    if (body == nullptr || orientations == nullptr || isBodyTracked(body) == false) {
        return E_FAIL;
    }

    return body->GetJointOrientations(JointType_Count, orientations);
}

HRESULT GePoTool::getJointOrientations(const BodyIndex &bodyIndex, JointOrientation *orientations) const
{
    return getJointOrientations(getBody(bodyIndex), orientations);
}

CameraSpacePoint GePoTool::getJointPosition(IBody *body, JointType joint) const
{
    CameraSpacePoint position = {0, 0, 0};
    if (!body) {
        return position;
    }

    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        position = joints[joint].Position;
    }
    return position;
}

CameraSpacePoint GePoTool::getJointPosition(const BodyIndex &bodyIndex, JointType joint) const
{
    return getJointPosition(getBody(bodyIndex), joint);
}

HRESULT GePoTool::getJoints(IBody *body, Joint *joints) const
{
    if (body == nullptr || joints == nullptr || isBodyTracked(body) == false) {
        return E_FAIL;
    }

    return body->GetJoints(JointType_Count, joints);
}

HRESULT GePoTool::getJoints(const BodyIndex &bodyIndex, Joint *joints) const
{
    return getJoints(getBody(bodyIndex), joints);
}

float GePoTool::getAngleBetweenTwoPoints(const CameraSpacePoint &pointOne, const CameraSpacePoint &pointTwo) const
{
    const float x = std::abs(pointOne.X - pointTwo.X);
    const float h = getDistanceBetweenPoints(pointOne, pointTwo);

    float angle = toDegree(acosf(x / h));
    // Without the following process, angle would only give results between 0 and 180. With this, we convert that to a 360 degree.
    if (pointOne.Y < pointTwo.Y) {
        angle *= -1;
    }
    if (pointOne.X > pointTwo.X) {
        angle = 180.f - angle;
    }

    return angle;
}

JointType GePoTool::getActiveHand(Joint *joints) const
{
    JointType joint = JointType_Count;
    // Set the higher hand as the active one
    if (joints[JointType_HandLeft].Position.Y > joints[JointType_HandRight].Position.Y) {
        joint = JointType_HandLeft;
    }
    else {
        joint = JointType_HandRight;
    }
    return joint;
}

IBody *GePoTool::getClosestBody()
{
    const int index = getBodyIndexWithTrackingID(m_KinectHandler->getClosestBodyID());
    if (index < 0) {
        return nullptr;
    }

    return m_Bodies[index];
}

bool GePoTool::isBodyRestricted(IBody *body) const
{
    if (!body) {
        return false;
    }

    BOOLEAN isRestricted = 0;
    HRESULT hr = body->get_IsRestricted(&isRestricted);
    if (SUCCEEDED(hr)) {
        return isRestricted == 1;
    }

    return false;
}

bool GePoTool::isBodyTracked(IBody *body) const
{
    if (body) {
        BOOLEAN isTracked = 0;
        body->get_IsTracked(&isTracked);
        return isTracked == 1;
    }

    return false;
}

void GePoTool::resetBodyNotification()
{
    onBodyLost = std::bind([](const BodyIndex & player) {}, std::placeholders::_1);
    onBodyFound = std::bind([](const BodyIndex & player) {}, std::placeholders::_1);
}
