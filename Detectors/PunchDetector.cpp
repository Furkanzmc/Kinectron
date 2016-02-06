#include "PunchDetector.h"
#include "GePoTool.h"

DPunchRight::DPunchRight(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DPunchRight", customID)
    , m_PostureTool(postureTool)
    , m_TimeAccumulatorRightPunch(0)
    , m_TimeAccumulatorLeftPunch(0)
    , m_DidLeftPunchFront(false)
    , m_DidRightPunchFront(false)
{
    memset(&m_TempPosRightPunch, 0, sizeof(CameraSpacePoint));
    memset(&m_TempPosLeftPunch, 0, sizeof(CameraSpacePoint));
}

UID DPunchRight::detect(IBody *body, const float &delta)
{
    if (!body) {
        return GePoTool::INVALID_UID;
    }

    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        const float requiredMaxTimeForPunch = .5f;
        const float requiredMinTimeForPunch = .2f;

        const CameraSpacePoint rightHandPos = joints[JointType_HandRight].Position;
        const CameraSpacePoint spineBasePos = joints[JointType_SpineBase].Position;
        const float shoulderLenght = std::abs(joints[JointType_ShoulderLeft].Position.X - joints[JointType_ShoulderRight].Position.X);
        float &timeAccumulator = m_TimeAccumulatorRightPunch;
        CameraSpacePoint &startPos = m_TempPosRightPunch;
        bool &didRightPunch = m_DidRightPunchFront;

        if (rightHandPos.Y < spineBasePos.Y) {
            timeAccumulator = 0;
            startPos = {0, 0, 0};
            didRightPunch = false;
            return GePoTool::INVALID_UID;
        }

        if (didRightPunch == false) {
            if (timeAccumulator == 0) {
                startPos = rightHandPos;
            }
            timeAccumulator += delta;

            if (timeAccumulator <= requiredMaxTimeForPunch && timeAccumulator >= requiredMinTimeForPunch) {
                //The Punch should be in front of the body
                if (rightHandPos.Z >= spineBasePos.Z) {
                    timeAccumulator = 0;
                    startPos = {0, 0, 0};
                    didRightPunch = false;
                    return GePoTool::INVALID_UID;
                }
                if (std::abs(rightHandPos.Z - startPos.Z) >= shoulderLenght / 2) {
                    didRightPunch = true;
                    timeAccumulator = 0;
                    startPos = {0, 0, 0};
                    return getID();
                }
                else {
                    timeAccumulator = 0;
                    startPos = {0, 0, 0};
                    didRightPunch = false;
                }
            }
            else if (timeAccumulator > requiredMaxTimeForPunch) {
                timeAccumulator = 0;
                startPos = {0, 0, 0};
                didRightPunch = false;
            }
        }
        else {
            if (timeAccumulator == 0 && std::abs(rightHandPos.Z - spineBasePos.Z) <= shoulderLenght / 2) {
                didRightPunch = false;
                timeAccumulator = 0;
                startPos = {0, 0, 0};
                return GePoTool::INVALID_UID;
            }
        }
    }
    return GePoTool::INVALID_UID;
}

//Detect left punch
DPunchLeft::DPunchLeft(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DPunchLeft", customID)
    , m_PostureTool(postureTool)
    , m_TimeAccumulatorRightPunch(0)
    , m_TimeAccumulatorLeftPunch(0)
    , m_DidLeftPunchFront(false)
    , m_DidRightPunchFront(false)
{
    memset(&m_TempPosRightPunch, 0, sizeof(CameraSpacePoint));
    memset(&m_TempPosLeftPunch, 0, sizeof(CameraSpacePoint));
}

UID DPunchLeft::detect(IBody *body, const float &delta)
{
    if (!body) {
        return GePoTool::INVALID_UID;
    }

    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        const float requiredMaxTimeForPunch = .5f;
        const float requiredMinTimeForPunch = .2f;

        const CameraSpacePoint leftHandPos = joints[JointType_HandLeft].Position;
        const CameraSpacePoint spineBasePos = joints[JointType_SpineBase].Position;
        float &timeAccumulator = m_TimeAccumulatorLeftPunch;
        CameraSpacePoint &startPos = m_TempPosLeftPunch;
        const float shoulderLenght = std::abs(joints[JointType_ShoulderLeft].Position.X - joints[JointType_ShoulderRight].Position.X);
        bool &didLeftPunch = m_DidLeftPunchFront;

        if (leftHandPos.Y < spineBasePos.Y) {
            timeAccumulator = 0;
            startPos = {0, 0, 0};
            didLeftPunch = false;
            return GePoTool::INVALID_UID;
        }

        if (didLeftPunch == false) {
            if (timeAccumulator == 0) {
                startPos = leftHandPos;
            }
            timeAccumulator += delta;

            if (timeAccumulator <= requiredMaxTimeForPunch && timeAccumulator >= requiredMinTimeForPunch) {
                //The Punch should be in front of the body
                if (leftHandPos.Z >= spineBasePos.Z) {
                    timeAccumulator = 0;
                    startPos = {0, 0, 0};
                    didLeftPunch = false;
                    return GePoTool::INVALID_UID;
                }
                if (std::abs(leftHandPos.Z - startPos.Z) >= shoulderLenght / 2) {
                    didLeftPunch = true;
                    timeAccumulator = 0;
                    startPos = {0, 0, 0};
                    return getID();
                }
                else {
                    timeAccumulator = 0;
                    startPos = {0, 0, 0};
                    didLeftPunch = false;
                }
            }
            else if (timeAccumulator > requiredMaxTimeForPunch) {
                timeAccumulator = 0;
                startPos = {0, 0, 0};
                didLeftPunch = false;
            }
        }
        else {
            if (timeAccumulator == 0 && std::abs(leftHandPos.Z - spineBasePos.Z) <= shoulderLenght / 2) {
                didLeftPunch = false;
                timeAccumulator = 0;
                startPos = {0, 0, 0};
                return GePoTool::INVALID_UID;
            }
        }
    }
    return GePoTool::INVALID_UID;
}
