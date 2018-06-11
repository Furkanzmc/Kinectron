#include "Kinectron/Detectors/KickDetector.h"
// Local
#include "Kinectron/GePoTool.h"

DKickRight::DKickRight(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DKickRight", customID)
    , m_PostureTool(postureTool)
    , m_TimeAccumulatorKickLeft(0)
    , m_TimeAccumulatorKickRight(0)
    , m_DidKickLeft(false)
    , m_DidKickRight(false)
{
    //memset the structs with no default constructor
    memset(&m_KickLeftStartPos, 0, sizeof(CameraSpacePoint));
    memset(&m_KickRightStartPos, 0, sizeof(CameraSpacePoint));
}

UID DKickRight::detect(IBody *body, const float &delta)
{
    if (!body) {
        return GePoTool::INVALID_UID;
    }

    bool isKick = false;
    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        const float requiredMaxTimeForKick = .5f;
        const float requiredMinTimeForKick = .2f;

        const CameraSpacePoint rightFootPos = joints[JointType_FootRight].Position;
        const CameraSpacePoint spineMidPos = joints[JointType_SpineMid].Position;
        float &timeAccumulator = m_TimeAccumulatorKickRight;
        CameraSpacePoint &startPos = m_KickRightStartPos;
        bool &didKick = m_DidKickRight;

        if (didKick == false) {
            if (timeAccumulator == 0) {
                startPos = rightFootPos;
            }
            timeAccumulator += delta;

            if (timeAccumulator <= requiredMaxTimeForKick && timeAccumulator >= requiredMinTimeForKick) {
                //The kick should be in front of the body
                if (rightFootPos.Z >= spineMidPos.Z) {
                    timeAccumulator = 0;
                    startPos = {0};
                    didKick = false;
                    return GePoTool::INVALID_UID;
                }
                if (rightFootPos.Z - startPos.Z <= -.25f) {
                    didKick = true;
                    timeAccumulator = 0;
                    startPos = {0};
                    return getID();
                }
                else {
                    timeAccumulator = 0;
                    startPos = {0};
                    didKick = false;
                }
            }
            else if (timeAccumulator > requiredMaxTimeForKick) {
                timeAccumulator = 0;
                startPos = {0};
                didKick = false;
            }
        }
        else {
            if (timeAccumulator == 0 && std::abs(rightFootPos.Z - spineMidPos.Z) <= .45f) {
                didKick = false;
                timeAccumulator = 0;
                startPos = {0};
                return GePoTool::INVALID_UID;
            }
        }
    }
    return isKick;
}

//Detect left kick
DKickLeft::DKickLeft(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DKickLeft", customID)
    , m_PostureTool(postureTool)
    , m_TimeAccumulatorKickLeft(0)
    , m_TimeAccumulatorKickRight(0)
    , m_DidKickLeft(false)
    , m_DidKickRight(false)
{
    //memset the structs with no default constructor
    memset(&m_KickLeftStartPos, 0, sizeof(CameraSpacePoint));
    memset(&m_KickRightStartPos, 0, sizeof(CameraSpacePoint));
}

UID DKickLeft::detect(IBody *body, const float &delta)
{
    if (!body) {
        return GePoTool::INVALID_UID;
    }

    bool isKick = false;
    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        const float requiredMaxTimeForKick = .5f;
        const float requiredMinTimeForKick = .2f;

        const CameraSpacePoint leftFootPos = joints[JointType_FootLeft].Position;
        const CameraSpacePoint spineMidPos = joints[JointType_SpineMid].Position;
        float &timeAccumulator = m_TimeAccumulatorKickLeft;
        CameraSpacePoint &startPos = m_KickLeftStartPos;
        bool &didKick = m_DidKickLeft;

        if (didKick == false) {
            if (timeAccumulator == 0) {
                startPos = leftFootPos;
            }
            timeAccumulator += delta;

            if (timeAccumulator <= requiredMaxTimeForKick && timeAccumulator >= requiredMinTimeForKick) {
                //The kick should be in front of the body
                if (leftFootPos.Z >= spineMidPos.Z) {
                    timeAccumulator = 0;
                    didKick = false;
                    startPos = {0};
                    return GePoTool::INVALID_UID;
                }
                if (leftFootPos.Z - startPos.Z <= -.25f) {
                    didKick = true;
                    timeAccumulator = 0;
                    startPos = {0};
                    return getID();
                }
                else {
                    timeAccumulator = 0;
                    startPos = {0};
                    didKick = false;
                }
            }
            else if (timeAccumulator > requiredMaxTimeForKick) {
                timeAccumulator = 0;
                startPos = {0};
                didKick = false;
            }
        }
        else {
            if (timeAccumulator == 0 && std::abs(leftFootPos.Z - spineMidPos.Z) <= .45f) {
                didKick = false;
                timeAccumulator = 0;
                startPos = {0};
                return GePoTool::INVALID_UID;
            }
        }
    }
    return isKick;
}
