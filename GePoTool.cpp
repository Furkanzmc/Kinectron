#include "GePoTool.h"
#include <iostream>
#include <string>

GePoTool::GePoTool(const unsigned int &sensorIniteType)
    : m_KinectHandler()
    , m_SensorTime(0)
{
    std::fill(m_Bodies.begin(), m_Bodies.end(), nullptr);
    m_KinectHandler.initializeDefaultSensor(sensorIniteType);
    m_KinectHandler.m_ProcessBodyFunc = std::bind(&GePoTool::processBody, this, std::placeholders::_1, std::placeholders::_2);
}

GePoTool::~GePoTool()
{
}

void GePoTool::processBody(const std::array<IBody *, BODY_COUNT> &bodyArray, UINT64 sensorTime)
{
    m_SensorTime = sensorTime;
    bodyNotificitons(bodyArray);
    //Copy the new skeletons
    m_Bodies = bodyArray;
}

void GePoTool::bodyNotificitons(const std::array<IBody *, BODY_COUNT> &bodyArray)
{
    auto countVisible = [](IBody * b) {
        return b != nullptr;
    };
    const unsigned int previousBodyCount = std::count_if(m_Bodies.begin(), m_Bodies.end(), countVisible);
    const unsigned int newBodyCount = std::count_if(bodyArray.begin(), bodyArray.end(), countVisible);

    if (newBodyCount == 0) {
        if (m_BodyLostFunc) {
            m_BodyLostFunc(PLAYER::ONE);
            m_BodyLostFunc(PLAYER::TWO);
            m_BodyLostFunc(PLAYER::THREE);
            m_BodyLostFunc(PLAYER::FOUR);
            m_BodyLostFunc(PLAYER::FIVE);
            m_BodyLostFunc(PLAYER::SIX);
        }
        return;
    }
    //If the m_Bodies is empty, copy the new vector and call the body found functions
    else if (previousBodyCount == 0) {
        for (unsigned int i = 0; i < newBodyCount; i++) {
            if (m_BodyFoundFunc) {
                m_BodyFoundFunc(i);
            }
        }
        return;
    }
    //If there are already items, and new bodyCount is bigger than the m_Bodies then new players joined.
    else if (newBodyCount > previousBodyCount) {
        for (unsigned int i = previousBodyCount - 1; i < newBodyCount; i++) {
            if (m_BodyFoundFunc) {
                m_BodyFoundFunc(i);
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
                if (bodyArray.at(newBodyIndex)) {
                    continue;
                }

                if (getTrackingID(m_Bodies.at(oldBodyIndex)) == getTrackingID(bodyArray.at(newBodyIndex))) {
                    isBodyAlreadyExist = true;
                    //Yeap, he's ok.
                    break;
                }
            }
            //If the body does not exist, it can only mean one thing... INFIDEL!!!
            if (isBodyAlreadyExist == false) {
                if (m_BodyLostFunc) {
                    m_BodyLostFunc(oldBodyIndex);
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

    int indexToRemove = -1;
    for (unsigned int i = 0; i < m_Detectors.size(); i++) {
        if (m_Detectors.at(i)->getUniqueID() == detector->getUniqueID()) {
            indexToRemove = i;
            break;
        }
    }
    if (indexToRemove > -1) {
        m_Detectors.erase(m_Detectors.begin() + indexToRemove);
        delete detector;
        detector = nullptr;
        return true;
    }
    return false;
}

KinectHandler &GePoTool::getKinectHandler()
{
    return m_KinectHandler;
}

IBody *GePoTool::getBody(const BodyIndex &player)
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

bool GePoTool::isBodyTracked(IBody *body) const
{
    if (body) {
        BOOLEAN isTracked = 0;
        body->get_IsTracked(&isTracked);
        return isTracked == 1;
    }
    return false;
}

UINT64 GePoTool::getTrackingID(IBody *body)
{
    if (!body) {
        return 0;
    }

    UINT64 trackingID = 0;
    body->get_TrackingId(&trackingID);
    return trackingID;
}

std::pair<HandState, HandState> GePoTool::getHandStates(IBody *body)
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
        PointF invalidLean{0};
        return invalidLean;
    }

    PointF leanAmount = {0};
    body->get_Lean(&leanAmount);
    return leanAmount;
}

float GePoTool::getAngleBetweenHands(IBody *body) const
{
    if (!body || isBodyTracked(body) == false) {
        return INVALID_ANGLE;
    }

    Joint joints[JointType_Count];
    body->GetJoints(_countof(joints), joints);

    CameraSpacePoint leftHand = joints[JointType_HandLeft].Position;
    CameraSpacePoint leftElbow = joints[JointType_ElbowLeft].Position;
    CameraSpacePoint leftShoulder = joints[JointType_ShoulderLeft].Position;

    CameraSpacePoint rightHand = joints[JointType_HandRight].Position;
    CameraSpacePoint rightElbow = joints[JointType_ElbowRight].Position;
    CameraSpacePoint rightShoulder = joints[JointType_ShoulderRight].Position;

    /* When getting the angle value, to prevent the angle detectin from happenning when the hands are just dangling the hand that is above the other
     * should also be over or on the elbow.
     */
    if (leftHand.Y > rightHand.Y && leftElbow.Y > leftHand.Y) {
        return INVALID_ANGLE;
    }
    if (rightHand.Y > leftHand.Y && rightElbow.Y > rightHand.Y) {
        return INVALID_ANGLE;
    }
    if (rightHand.X < rightShoulder.X) {
        return INVALID_ANGLE;
    }
    if (leftHand.X > leftShoulder.X) {
        return INVALID_ANGLE;
    }

    //x -> Distance on x axis, y -> distance on y axis, h -> hypotenuse
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
    float angle = (radian * 180) / 3.14159265f;//Convert to degrees
    if (leftHand.Y < rightHand.Y) {
        angle *= -1;
    }
    return angle;
}

std::vector<UID> GePoTool::pollGestures(const BodyIndex &bodyIndex, const float &delta)
{
    std::vector<UID> detectedGestures;
    for (auto detector : m_Detectors) {
        if (detector) {
            //If the detector already is set to a body index, and the given body inex does not eqaul to it skip the detector.
            if (detector->getBodyIndex() > DGestureBase::INVALID_BODY_INDEX && detector->getBodyIndex() != bodyIndex) {
                continue;
            }

            IBody *body = getBody(bodyIndex);
            if (!body) {
                continue;
            }

            const UID &uid = detector->detect(body, delta);
            if (uid == detector->getID()) {
                detectedGestures.push_back(uid);
            }
        }
    }
    if (detectedGestures.size() > 1) {
        //Sort in ascending order
        std::sort(detectedGestures.begin(), detectedGestures.end());
    }
    return detectedGestures;
}

UID GePoTool::determinePlayerPosture(const BodyIndex &bodyIndex, const float &delta)
{
    UID detectedGesture = GePoTool::INVALID_UID;
    for (auto detector : m_Detectors) {
        if (detector) {
            //If the detector already is set to a body index, and the given body inex does not eqaul to it skip the detector.
            if (detector->getBodyIndex() > DGestureBase::INVALID_BODY_INDEX && detector->getBodyIndex() != bodyIndex) {
                continue;
            }

            IBody *body = getBody(bodyIndex);
            if (!body) {
                continue;
            }

            const UID &uid = detector->detect(body, delta);
            if (uid == detector->getID()) {
                detectedGesture = uid;
                break;
            }
        }
    }
    return detectedGesture;
}

unsigned int GePoTool::getDetectedBodyCount() const
{
    auto countVisible = [](IBody * b) {
        return b != nullptr;
    };

    return std::count_if(m_Bodies.begin(), m_Bodies.end(), countVisible);
}

GePoTool::BodyRect GePoTool::getBodyRect(IBody *body)
{
    if (!body) {
        return BodyRect();
    }

    const int HEAD = 0, HAND_RIGHT = 1, HAND_LEFT = 2, FOOT_RIGHT = 3, FOOT_LEFT = 4, SHOULDER_RIGHT = 5, SHOULDER_LEFT = 6, ELBOW_LEFT = 7, ELBOW_RIGHT = 8;
    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        ColorSpacePoint colorPoints[9] = {0};
        CameraSpacePoint cameraPoints[9] = {0};
        //TODO: Prevent the left and right from being smaller than shoulders
        //TODO: Get the distance between shoulder mid and head, add that distance to the rect
        //FIXME
        //This is mapped for 1920x1080 resolution
        cameraPoints[HEAD] = joints[JointType_Head].Position;
        cameraPoints[HAND_RIGHT] = joints[JointType_HandRight].Position;
        cameraPoints[HAND_LEFT] = joints[JointType_HandLeft].Position;
        cameraPoints[FOOT_RIGHT] = joints[JointType_FootRight].Position;
        cameraPoints[FOOT_LEFT] = joints[JointType_FootLeft].Position;
        cameraPoints[SHOULDER_RIGHT] = joints[JointType_ShoulderRight].Position;
        cameraPoints[SHOULDER_LEFT] = joints[JointType_ShoulderLeft].Position;
        cameraPoints[ELBOW_LEFT] = joints[JointType_ElbowLeft].Position;
        cameraPoints[ELBOW_RIGHT] = joints[JointType_ElbowLeft].Position;

        hr = m_KinectHandler.getCoordinateMapper()->MapCameraPointsToColorSpace(_countof(cameraPoints), cameraPoints, _countof(colorPoints), colorPoints);
        if (SUCCEEDED(hr)) {
            colorPoints[HEAD].Y -= 100;
            colorPoints[HAND_RIGHT].X += 50;
            colorPoints[HAND_LEFT].X -= 50;
            colorPoints[FOOT_RIGHT].X += 50;
            colorPoints[FOOT_LEFT].X -= 50;
            //            colorPoints[FOOT_LEFT].Y += 50;
            colorPoints[SHOULDER_RIGHT].X += 50;
            colorPoints[SHOULDER_LEFT].X -= 50;
            colorPoints[ELBOW_LEFT].X -= 50;
            colorPoints[ELBOW_RIGHT].Y += 50;
            const float shoulderLenght = std::abs(colorPoints[SHOULDER_LEFT].X - colorPoints[SHOULDER_RIGHT].X) + 50;

            BodyRect rect;
            rect.x = colorPoints[HAND_LEFT].X;
            rect.y = colorPoints[HEAD].Y;
            rect.width = std::abs(colorPoints[HAND_LEFT].X - colorPoints[HAND_RIGHT].X);
            rect.height = std::abs(colorPoints[HEAD].Y - colorPoints[FOOT_LEFT].Y);
            //If the hands get closer, take the shoulder as lenght as the rect width
            if (rect.width < shoulderLenght) {
                rect.width = shoulderLenght;
            }
            //If the legs are wider than the shoulder or hands distance
            if (rect.width < std::abs(colorPoints[FOOT_LEFT].X - colorPoints[FOOT_RIGHT].X)) {
                rect.width = std::abs(colorPoints[FOOT_LEFT].X - colorPoints[FOOT_RIGHT].X);
            }
            //If the left hand goes to the right side of the shoulder, take the shoulder as rect.x
            if (rect.x > colorPoints[SHOULDER_LEFT].X) {
                rect.x = colorPoints[SHOULDER_LEFT].X - 50;
            }
            //If the left foot goes beyond left hand or left shoulder
            if (colorPoints[FOOT_LEFT].X < rect.x) {
                rect.x = colorPoints[FOOT_LEFT].X;
            }
            //If the right foot goes beyond right hand or right shoulder
            if (colorPoints[FOOT_RIGHT].X > colorPoints[SHOULDER_RIGHT].X || colorPoints[FOOT_RIGHT].X > colorPoints[SHOULDER_RIGHT].X) {
                rect.x = colorPoints[FOOT_LEFT].X;
            }
            //If the right hand go above the head
            if (rect.y > colorPoints[HAND_RIGHT].Y) {
                rect.y = colorPoints[HAND_RIGHT].Y;
            }
            //If the left hand go above the head
            if (rect.y > colorPoints[HAND_LEFT].Y) {
                rect.y = colorPoints[HAND_LEFT].Y;
            }
            //If the head position is below zero, make it 0
            if (rect.y < 0) {
                rect.y = 0;
            }
            return rect;
        }
    }
    return BodyRect();
}

GePoTool::BodyRect GePoTool::getHeadRect(IBody *body)
{
    if (!body) {
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
        //This is mapped for 1920x1080 resolution
        cameraPoints[headIndex] = joints[JointType_Head].Position;
        cameraPoints[shoulderRightIndex] = joints[JointType_ShoulderRight].Position;
        cameraPoints[shoulderLeftIndex] = joints[JointType_ShoulderLeft].Position;

        colorPoints[headIndex] = m_KinectHandler.mapBodyPointToScreenPoint(cameraPoints[headIndex]);
        colorPoints[shoulderRightIndex] = m_KinectHandler.mapBodyPointToScreenPoint(cameraPoints[shoulderRightIndex]);
        colorPoints[shoulderLeftIndex] = m_KinectHandler.mapBodyPointToScreenPoint(cameraPoints[shoulderLeftIndex]);

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
    return (radian * 180) / 3.14158265f;
}

float GePoTool::toRadian(const float &degree) const
{
    return (degree * 3.14159265f) / 180;
}

Vector4 GePoTool::getJointOrientation(IBody *body, JointType joint)
{
    if (!body) {
        return Vector4();
    }

    JointOrientation joints[JointType_Count];
    body->GetJointOrientations(_countof(joints), joints);
    return joints[joint].Orientation;
}

CameraSpacePoint GePoTool::getJointPosition(IBody *body, JointType joint)
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

float GePoTool::getAngleBetweenTwoPoints(const CameraSpacePoint &pointOne, const CameraSpacePoint &pointTwo)
{
    float x = 0, h = getDistanceBetweenPoints(pointOne, pointTwo);
    x = std::abs(pointOne.X - pointTwo.X);
    float angle = toDegree(acosf(x / h));
    //Without the following process, angle would only give results between 0 and 180. With this, we convert that to a 360 degree.
    if (pointOne.Y < pointTwo.Y) {
        angle *= -1;
    }
    if (pointOne.X > pointTwo.X) {
        angle = 180 - angle;
    }
    return angle;
}

JointType GePoTool::getActiveHand(Joint *joints)
{
    JointType joint = JointType_Count;
    //Set the higher hand as the active one
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
    const int index = getBodyIndexWithTrackingID(m_KinectHandler.getClosestBodyID());
    if (index < 0) {
        return nullptr;
    }

    return m_Bodies[index];
}

void GePoTool::resetBodyNotification()
{
    m_BodyLostFunc = std::bind([](const BodyIndex & player) {}, std::placeholders::_1);
    m_BodyFoundFunc = std::bind([](const BodyIndex & player) {}, std::placeholders::_1);
}
