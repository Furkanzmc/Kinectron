#include "ArmWaveDetector.h"
#include "GePoTool.h"

UID ArmWaveDetectorHelper::detectArmWaveGesture(IBody *body, const GESTURE_SIDE &gestureSide, ArmWaveInfo &waveInfo, const float &delta, const UID &uid)
{
    if (!body) {
        return GePoTool::INVALID_UID;
    }

    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        auto resetInfo = [](ArmWaveInfo & info) {
            info.isHandAboveHead = false;
            info.isHandAboveShoulder = false;
            info.isElbowAboveHead = false;
            info.isElbowAboveShoulder = false;
            info.timeAccumulator = 0;
            info.isGestureDone = false;
        };

        const CameraSpacePoint handPos = gestureSide == GESTURE_SIDE::RIGHT ? joints[JointType_HandRight].Position : joints[JointType_HandLeft].Position;
        const CameraSpacePoint elbowPos = gestureSide == GESTURE_SIDE::RIGHT ? joints[JointType_ElbowRight].Position : joints[JointType_ElbowLeft].Position;
        const CameraSpacePoint headPos = joints[JointType_Head].Position;
        const CameraSpacePoint spineBasePos = joints[JointType_SpineBase].Position;
        const CameraSpacePoint shoulderPos = gestureSide == GESTURE_SIDE::RIGHT ? joints[JointType_ShoulderRight].Position : joints[JointType_ShoulderLeft].Position;
        //If the hand is below the shoulder or the hip, return false
        if (handPos.Y < shoulderPos.Y || handPos.Y < spineBasePos.Y) {
            resetInfo(waveInfo);
            return GePoTool::INVALID_UID;
        }
        //If the hand is on the left side of the elbow before the hand is over the shoulder, return false
        const bool isHandExceedElbow = gestureSide == GESTURE_SIDE::RIGHT ? handPos.X < elbowPos.X : handPos.X > elbowPos.X;
        if (isHandExceedElbow && waveInfo.isHandAboveShoulder == false) {
            resetInfo(waveInfo);
            return GePoTool::INVALID_UID;
        }
        //If the hand is over the shoulder, start scanning for the gesture
        if (handPos.Y >= shoulderPos.Y && waveInfo.isGestureDone == false) {
            waveInfo.isHandAboveShoulder = true;
            waveInfo.timeAccumulator += delta;
            if (elbowPos.Y >= shoulderPos.Y) {
                waveInfo.isElbowAboveShoulder = true;
            }
            else {
                waveInfo.isElbowAboveShoulder = false;
            }
            if (handPos.Y >= headPos.Y) {
                waveInfo.isHandAboveHead = true;
            }
            else {
                waveInfo.isHandAboveHead = false;
            }
            //If the elbow is over the shoulder and the hand is above the head, check for the gesture successs
            if (waveInfo.isElbowAboveShoulder && waveInfo.isHandAboveHead) {
                if (waveInfo.timeAccumulator >= waveInfo.minimalTime && waveInfo.timeAccumulator <= waveInfo.maximumTime) {
                    if (handPos.X <= elbowPos.X) {
                        waveInfo.isGestureDone = true;
                        waveInfo.timeAccumulator = 0;
                    }
                    return GePoTool::INVALID_UID;
                }
                else if (waveInfo.timeAccumulator > waveInfo.maximumTime) {
                    resetInfo(waveInfo);
                    return GePoTool::INVALID_UID;
                }
            }
        }
        //If the gesture is done, wait for the hand to return below shoulder o start scanning again
        if (waveInfo.isGestureDone) {
            waveInfo.timeAccumulator += delta;
            if (elbowPos.Y < shoulderPos.Y) {
                resetInfo(waveInfo);
                return uid;
            }
            else if (waveInfo.timeAccumulator > waveInfo.maximumTime * 1.5f) {
                resetInfo(waveInfo);
                return GePoTool::INVALID_UID;
            }
        }
    }
    return GePoTool::INVALID_UID;
}

//Detect left arm wave
DArmWaveLeft::DArmWaveLeft(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DArmWaveLeft", customID)
    , m_PostureTool(postureTool)
    , m_ArmWaveInfo()
{

}

UID DArmWaveLeft::detect(IBody *body, const float &delta)
{
    return ArmWaveDetectorHelper::detectArmWaveGesture(body, GESTURE_SIDE::LEFT, m_ArmWaveInfo, delta, getID());
}

//Detect right arm wave
DArmWaveRight::DArmWaveRight(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DArmWaveRight", customID)
    , m_PostureTool(postureTool)
    , m_ArmWaveInfo()
{

}

UID DArmWaveRight::detect(IBody *body, const float &delta)
{
    return ArmWaveDetectorHelper::detectArmWaveGesture(body, GESTURE_SIDE::RIGHT, m_ArmWaveInfo, delta, getID());
}

