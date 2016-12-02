#include "Kinectron/Detectors/SwipeDetector.h"
// Local
#include "Kinectron/GePoTool.h"

DSwipeLeft::DSwipeLeft(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DSwipeLeft", customID)
    , m_PostureTool(postureTool)
      //Swipe
    , m_MinimalTimeForSwipe(.1f)
    , m_MaximumTimeForSwipe(.7f)
      //Swipe Left
    , m_TimeAccumulatorSwipe(0.f)
    , m_IsHandRightOnRight(false)
{

}

UID DSwipeLeft::detect(IBody *body, const float &delta)
{
    const HandState leftHandState = m_PostureTool.getHandStates(body).first;
    if (!body || (leftHandState == HandState_Closed)) {
        return GePoTool::INVALID_UID;
    }

    bool isSwipe = false;
    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        bool &isOnRight = m_IsHandRightOnRight;
        float &timeAccumulator = m_TimeAccumulatorSwipe;

        const CameraSpacePoint rightHandPos = joints[JointType_HandRight].Position;
        const CameraSpacePoint rightElbowPos = joints[JointType_ElbowRight].Position;
        const CameraSpacePoint shoulderPos = joints[JointType_ShoulderRight].Position;
        const CameraSpacePoint hipRightPos = joints[JointType_HipRight].Position;
        const CameraSpacePoint shoulderRightPos = joints[JointType_ShoulderRight].Position;
        const CameraSpacePoint spineMidPos = joints[JointType_SpineMid].Position;

        auto resetValues = [&]() {
            bool &isOnRight = m_IsHandRightOnRight;
            float &timeAccumulator = m_TimeAccumulatorSwipe;
            isOnRight = false;
            timeAccumulator = 0;
        };

        //If the hand is below elbow, now swipe gesture
        if (rightHandPos.Y < rightElbowPos.Y) {
            resetValues();
            return GePoTool::INVALID_UID;
        }
        //If the hand is above the shoulder, no swipe gesture
        if (rightHandPos.Y > shoulderPos.Y) {
            resetValues();
            return GePoTool::INVALID_UID;
        }
        //If the hand is below the hip, no swipe gesture
        if (rightHandPos.Y < hipRightPos.Y) {
            resetValues();
            return GePoTool::INVALID_UID;
        }
        //If the hand is on the right of the right shoulder mark it
        if (rightHandPos.X >= shoulderRightPos.X) {
            isOnRight = true;
        }
        //If the hand is on the right side of the shoulder start scanning for the gesture
        if (isOnRight) {
            timeAccumulator += delta;
            if (timeAccumulator >= m_MinimalTimeForSwipe && timeAccumulator <= m_MaximumTimeForSwipe) {
                if (rightHandPos.X <= spineMidPos.X + std::abs(shoulderRightPos.X - spineMidPos.X) / 4) {
                    isSwipe = true;
                    resetValues();
                }
            }
            else if (timeAccumulator > m_MaximumTimeForSwipe) {
                resetValues();
            }
        }
    }
    return isSwipe ? getID() : GePoTool::INVALID_UID;
}

//Swipe right
DSwipeRight::DSwipeRight(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DSwipeRight", customID)
    , m_PostureTool(postureTool)
      //Swipe
    , m_MinimalTimeForSwipe(.1f)
    , m_MaximumTimeForSwipe(.7f)
      //Swipe Right
    , m_TimeAccumulatorSwipe(0.f)
    , m_IsHandLeftOnLeft(false)
{

}

UID DSwipeRight::detect(IBody *body, const float &delta)
{
    const HandState rightHandState = m_PostureTool.getHandStates(body).second;
    if (!body || (rightHandState == HandState_Closed)) {
        return GePoTool::INVALID_UID;
    }

    bool isSwipe = false;
    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        bool &isOnLeft = m_IsHandLeftOnLeft;
        float &timeAccumulator = m_TimeAccumulatorSwipe;

        const CameraSpacePoint leftHandPos = joints[JointType_HandLeft].Position;
        const CameraSpacePoint leftElbowPos = joints[JointType_ElbowLeft].Position;
        const CameraSpacePoint shoulderPos = joints[JointType_ShoulderLeft].Position;
        const CameraSpacePoint hipLeftPos = joints[JointType_HipLeft].Position;
        const CameraSpacePoint shoulderLeftPos = joints[JointType_ShoulderLeft].Position;
        const CameraSpacePoint spineMidPos = joints[JointType_SpineMid].Position;

        auto resetValues = [&]() {
            bool &isOnLeft = m_IsHandLeftOnLeft;
            float &timeAccumulator = m_TimeAccumulatorSwipe;
            isOnLeft = false;
            timeAccumulator = 0;
        };

        //If the hand is below elbow, now swipe gesture
        if (leftHandPos.Y < leftElbowPos.Y) {
            resetValues();
            return GePoTool::INVALID_UID;
        }
        //If the hand is above the shoulder, no swipe gesture
        if (leftHandPos.Y > shoulderPos.Y) {
            resetValues();
            return GePoTool::INVALID_UID;
        }
        //If the hand is below or over the hip, no swipe gesture
        if (leftHandPos.Y <= hipLeftPos.Y) {
            resetValues();
            return GePoTool::INVALID_UID;
        }
        //If the hand is on the left of the left shoulder mark it
        if (leftHandPos.X <= shoulderLeftPos.X) {
            isOnLeft = true;
        }
        //If the hand is on the left side of the shoulder start scanning for the gesture
        if (isOnLeft) {
            timeAccumulator += delta;
            if (timeAccumulator >= m_MinimalTimeForSwipe && timeAccumulator <= m_MaximumTimeForSwipe) {
                if (leftHandPos.X >= spineMidPos.X - std::abs(shoulderLeftPos.X - spineMidPos.X) / 4) {
                    resetValues();
                    isSwipe = true;
                }
            }
            else if (timeAccumulator > m_MaximumTimeForSwipe) {
                resetValues();
            }
        }
    }
    return isSwipe ? getID() : GePoTool::INVALID_UID;
}
