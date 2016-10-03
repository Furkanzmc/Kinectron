#include "OpenArmDetector.h"
// Local
#include "GePoTool.h"

DOpenArmLeft::DOpenArmLeft(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DOpenArmLeft", customID)
    , m_PostureTool(postureTool)
    , m_DurationLimit(.5f)
    , m_Duration(0.f)
    , m_IsGestureInitiated(false)
{

}

UID DOpenArmLeft::detect(IBody *body, const float &delta)
{
    if (!body) {
        return GePoTool::INVALID_UID;
    }

    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        //If hand , elbow and shoulder aligns together on the X axis then the arm is open
        const CameraSpacePoint handPos = joints[JointType_HandLeft].Position;
        const CameraSpacePoint elbowPos = joints[JointType_ElbowLeft].Position;
        const CameraSpacePoint shoulderPos = joints[JointType_ShoulderLeft].Position;
        const CameraSpacePoint headPos = joints[JointType_Head].Position;

        const CameraSpacePoint handRightPos = joints[JointType_HandRight].Position;
        const CameraSpacePoint elbowRightPos = joints[JointType_ElbowRight].Position;
        const CameraSpacePoint spineMidPos = joints[JointType_SpineMid].Position;

        //The left hand should be below the shoulder and the elbow
        if (handRightPos.Y > elbowRightPos.Y || handRightPos.Y > spineMidPos.Y) {
            return GePoTool::INVALID_UID;
        }

        //If the hand is not at least the half lenght of the shoulders, no posture
        if (std::abs(shoulderPos.X - headPos.X) > std::abs(handPos.X - shoulderPos.X)) {
            resetGesture();
            return GePoTool::INVALID_UID;
        }
        //If hand is too over elbow, return GePoTool::INVALID_UID
        if (handPos.Y - elbowPos.Y > .15f) {
            resetGesture();
            return GePoTool::INVALID_UID;
        }
        if (handPos.Y - elbowPos.Y < -.15f) {
            resetGesture();
            return GePoTool::INVALID_UID;
        }
        if (elbowPos.Y - shoulderPos.Y > .15f) {
            resetGesture();
            return GePoTool::INVALID_UID;
        }
        if (elbowPos.Y - shoulderPos.Y < -.15f) {
            resetGesture();
            return GePoTool::INVALID_UID;
        }
        if (elbowPos.X <= handPos.X) {
            resetGesture();
            return GePoTool::INVALID_UID;
        }
    }

    if (!m_IsGestureInitiated) {
        m_IsGestureInitiated = true;
    }
    else {
        m_Duration += delta;
        if (m_Duration >= m_DurationLimit) {
            resetGesture();
            return getID();
        }
    }
    return GePoTool::INVALID_UID;
}

void DOpenArmLeft::resetGesture()
{
    m_Duration = 0;
    m_IsGestureInitiated = false;
}

// ---------------------------------

DOpenArmRight::DOpenArmRight(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DOpenArmRight", customID)
    , m_PostureTool(postureTool)
    , m_DurationLimit(.5f)
    , m_Duration(0.f)
    , m_IsGestureInitiated(false)
{

}

UID DOpenArmRight::detect(IBody *body, const float &delta)
{
    if (!body) {
        return GePoTool::INVALID_UID;
    }

    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        //If hand , elbow and shoulder aligns together on the X axis then the arm is open
        const CameraSpacePoint handRightPos = joints[JointType_HandRight].Position;
        const CameraSpacePoint elbowRightPos = joints[JointType_ElbowRight].Position;
        const CameraSpacePoint shoulderRightPos = joints[JointType_ShoulderRight].Position;
        const CameraSpacePoint headPos = joints[JointType_Head].Position;

        const CameraSpacePoint handLeftPos = joints[JointType_HandLeft].Position;
        const CameraSpacePoint elbowLeftPos = joints[JointType_ElbowLeft].Position;
        const CameraSpacePoint spineMidPos = joints[JointType_SpineMid].Position;

        //The left hand should be below the shoulder and the elbow
        if (handLeftPos.Y > elbowLeftPos.Y || handLeftPos.Y > spineMidPos.Y) {
            return GePoTool::INVALID_UID;
        }

        //If the hand is not at least the half lenght of the shoulders, no posture
        if (std::abs(shoulderRightPos.X - headPos.X) > std::abs(handRightPos.X - shoulderRightPos.X)) {
            resetGesture();
            return GePoTool::INVALID_UID;
        }
        //If hand is too over elbow, return GePoTool::INVALID_UID
        if (handRightPos.Y - elbowRightPos.Y > .15f) {
            resetGesture();
            return GePoTool::INVALID_UID;
        }
        if (handRightPos.Y - elbowRightPos.Y < -.15f) {
            resetGesture();
            return GePoTool::INVALID_UID;
        }
        if (elbowRightPos.Y - shoulderRightPos.Y > .15f) {
            resetGesture();
            return GePoTool::INVALID_UID;
        }
        if (elbowRightPos.Y - shoulderRightPos.Y < -.15f) {
            resetGesture();
            return GePoTool::INVALID_UID;
        }
        if (elbowRightPos.X >= handRightPos.X) {
            resetGesture();
            return GePoTool::INVALID_UID;
        }
    }

    if (!m_IsGestureInitiated) {
        m_IsGestureInitiated = true;
    }
    else {
        m_Duration += delta;
        if (m_Duration >= m_DurationLimit) {
            resetGesture();
            return getID();
        }
    }
    return GePoTool::INVALID_UID;
}

void DOpenArmRight::resetGesture()
{
    m_Duration = 0;
    m_IsGestureInitiated = false;
}

// ---------------------------------

DOpenArms::DOpenArms(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DOpenArms", customID)
    , m_PostureTool(postureTool)
    , m_DurationLimit(.5f)
    , m_Duration(0.f)
    , m_IsGestureInitiated(false)
{

}

UID DOpenArms::detect(IBody *body, const float &delta)
{
    const bool isDetected = detectOpenLeftArm(body) && detectOpenRightArm(body);

    if (isDetected && !m_IsGestureInitiated) {
        m_IsGestureInitiated = true;
    }
    else if (isDetected) {
        m_Duration += delta;
        if (m_Duration >= m_DurationLimit) {
            resetGesture();
            return getID();
        }
    }
    else {
        resetGesture();
    }
    return GePoTool::INVALID_UID;
}

bool DOpenArms::detectOpenRightArm(IBody *body)
{
    if (!body) {
        return false;
    }

    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        //If hand , elbow and shoulder aligns together on the X axis then the arm is open
        const CameraSpacePoint handPos = joints[JointType_HandRight].Position;
        const CameraSpacePoint elbowPos = joints[JointType_ElbowRight].Position;
        const CameraSpacePoint shoulderPos = joints[JointType_ShoulderRight].Position;
        const CameraSpacePoint headPos = joints[JointType_Head].Position;

        //If the hand is not at least the half lenght of the shoulders, no posture
        if (std::abs(shoulderPos.X - headPos.X) > std::abs(handPos.X - shoulderPos.X)) {
            return GePoTool::INVALID_UID;
        }
        //If hand is too over elbow, return false
        if (handPos.Y - elbowPos.Y > .15f) {
            return false;
        }
        if (handPos.Y - elbowPos.Y < -.15f) {
            return false;
        }
        if (elbowPos.Y - shoulderPos.Y > .15f) {
            return false;
        }
        if (elbowPos.Y - shoulderPos.Y < -.15f) {
            return false;
        }
        if (elbowPos.X >= handPos.X) {
            return false;
        }
    }
    return true;
}

bool DOpenArms::detectOpenLeftArm(IBody *body)
{
    if (!body) {
        return false;
    }

    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        //If hand , elbow and shoulder aligns together on the X axis then the arm is open
        const CameraSpacePoint handPos = joints[JointType_HandLeft].Position;
        const CameraSpacePoint elbowPos = joints[JointType_ElbowLeft].Position;
        const CameraSpacePoint shoulderPos = joints[JointType_ShoulderLeft].Position;
        const CameraSpacePoint headPos = joints[JointType_Head].Position;

        //If the hand is not at least the half lenght of the shoulders, no posture
        if (std::abs(shoulderPos.X - headPos.X) > std::abs(handPos.X - shoulderPos.X)) {
            return false;
        }
        //If hand is too over elbow, return false
        if (handPos.Y - elbowPos.Y > .15f) {
            return false;
        }
        if (handPos.Y - elbowPos.Y < -.15f) {
            return false;
        }
        if (elbowPos.Y - shoulderPos.Y > .15f) {
            return false;
        }
        if (elbowPos.Y - shoulderPos.Y < -.15f) {
            return false;
        }
        if (elbowPos.X <= handPos.X) {
            return false;
        }
    }
    return true;
}

void DOpenArms::resetGesture()
{
    m_Duration = 0;
    m_IsGestureInitiated = false;
}
