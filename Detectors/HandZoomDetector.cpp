#include "HandZoomDetector.h"
// Local
#include "GePoTool.h"

UID HandZoomHelper::detectHandZoom(GePoTool &postureTool, IBody *body, const ZOOM_TYPE &zoomType, HandZoomInfo &zoomInfo, const float &delta, const UID &uid)
{
    if (!body || static_cast<int>(zoomType) > static_cast<int>(ZOOM_TYPE::ZOOM_OUT)) {
        return GePoTool::INVALID_UID;
    }

    auto resetInfo = [](HandZoomInfo & info) {
        info.startPos = {0};
        info.timeAccumulator = 0;
        info.scanStarted = false;
        info.isDoneGesture = false;
    };

    Joint joints[JointType_Count];
    HandState handState = HandState_Unknown;
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        CameraSpacePoint handPosition = {0};
        const CameraSpacePoint spineMidPos = joints[JointType_SpineMid].Position;
        const float shoulderLength = std::abs(joints[JointType_ShoulderLeft].Position.X - joints[JointType_ShoulderRight].Position.X);
        const std::pair<HandState, HandState> handStates = postureTool.getHandStates(body);
        //Set the higher hand as the active one
        if (joints[JointType_HandLeft].Position.Y > joints[JointType_HandRight].Position.Y) {
            handPosition = joints[JointType_HandLeft].Position;
            handState = handStates.first;
        }
        else {
            handPosition = joints[JointType_HandRight].Position;
            handState = handStates.second;
        }
        //If the hand is below the hip return GePoTool::INVALID_UID
        if (handPosition.Y < spineMidPos.Y) {
            resetInfo(zoomInfo);
            return GePoTool::INVALID_UID;
        }

        if (handState == HandState_Open) {
            resetInfo(zoomInfo);
        }
        else if (zoomInfo.scanStarted == false && handState == HandState_Closed && zoomInfo.isDoneGesture == false) {
            zoomInfo.startPos = handPosition;
            zoomInfo.scanStarted = true;
        }
        else if (zoomInfo.scanStarted && handState == HandState_Closed && zoomInfo.isDoneGesture == false) {
            zoomInfo.timeAccumulator += delta;
            if (zoomInfo.timeAccumulator >= zoomInfo.minimalTime && zoomInfo.timeAccumulator <= zoomInfo.maximumTime) {
                bool isSuccess = false;
                if (zoomType == ZOOM_TYPE::ZOOM_IN) {
                    isSuccess = handPosition.Z - zoomInfo.startPos.Z >= shoulderLength / 2;
                }
                else if (zoomType == ZOOM_TYPE::ZOOM_OUT) {
                    isSuccess = handPosition.Z - zoomInfo.startPos.Z <= -shoulderLength / 2;
                }

                if (isSuccess) {
                    zoomInfo.isDoneGesture = true;
                    zoomInfo.timeAccumulator = 0;
                    return uid;
                }
            }
            else if (zoomInfo.timeAccumulator > zoomInfo.maximumTime) {
                resetInfo(zoomInfo);
                return GePoTool::INVALID_UID;
            }
        }
        else if (zoomInfo.timeAccumulator == 0 && zoomInfo.isDoneGesture && handState == HandState_Open) {
            resetInfo(zoomInfo);
            return GePoTool::INVALID_UID;
        }
    }
    return GePoTool::INVALID_UID;
}

UID HandZoomHelper::detectHandPinchZoom(GePoTool &postureTool, IBody *body, const ZOOM_TYPE &zoomType, PinchZoomInfo &zoomInfo, const float &delta,
                                        const UID &uid)
{
    if (!body || static_cast<int>(zoomType) <= static_cast<int>(ZOOM_TYPE::ZOOM_OUT)) {
        return GePoTool::INVALID_UID;
    }

    auto resetInfo = [](PinchZoomInfo & info) {
        info.first.startPos = {0};
        info.first.timeAccumulator = 0;
        info.first.scanStarted = false;
        info.first.isDoneGesture = false;

        info.second.startPos = {0};
        info.second.timeAccumulator = 0;
        info.second.scanStarted = false;
        info.second.isDoneGesture = false;
    };

    auto isScanStarted = [](const PinchZoomInfo & pincInfo) {
        return pincInfo.first.scanStarted && pincInfo.second.scanStarted;
    };
    auto isHandsClosed = [](const HandState & handStateRight, const HandState & handStateLeft) {
        return handStateLeft == HandState_Closed && handStateRight == HandState_Closed;
    };
    auto isHandsOpen = [](const HandState & handStateRight, const HandState & handStateLeft) {
        return handStateLeft == HandState_Open && handStateRight == HandState_Open;
    };
    auto isDoneGesture = [](const PinchZoomInfo & pincInfo) {
        return pincInfo.first.isDoneGesture && pincInfo.second.isDoneGesture;
    };
    auto isTimeBetweenMinAndMax = [](const PinchZoomInfo & pinchZoomInfo) {
        return (pinchZoomInfo.first.timeAccumulator >= pinchZoomInfo.first.minimalTime && pinchZoomInfo.first.timeAccumulator <= pinchZoomInfo.first.maximumTime) &&
               (pinchZoomInfo.second.timeAccumulator >= pinchZoomInfo.second.minimalTime && pinchZoomInfo.second.timeAccumulator <= pinchZoomInfo.second.maximumTime);
    };
    auto isTimeAbovedMax = [](const PinchZoomInfo & pinchZoomInfo) {
        return (pinchZoomInfo.first.timeAccumulator > pinchZoomInfo.first.maximumTime) || (pinchZoomInfo.second.timeAccumulator > pinchZoomInfo.second.maximumTime);
    };

    Joint joints[JointType_Count];
    const std::pair<HandState, HandState> handStates = postureTool.getHandStates(body);
    const HandState handRightState = handStates.second;
    const HandState handLeftState = handStates.first;
    HandZoomInfo &zoomLeftInfo = zoomInfo.first;
    HandZoomInfo &zoomRightInfo = zoomInfo.second;
    HRESULT hr = body->GetJoints(_countof(joints), joints);

    if (SUCCEEDED(hr)) {
        const CameraSpacePoint handRightCurrentPosition = joints[JointType_HandRight].Position;
        const CameraSpacePoint handLeftCurrentPosition = joints[JointType_HandLeft].Position;
        const CameraSpacePoint spineMidPos = joints[JointType_SpineMid].Position;
        const float shoulderLength = std::abs(joints[JointType_ShoulderLeft].Position.X - joints[JointType_ShoulderRight].Position.X);
        //If the hands are below the hip and the gesture hadn't started return GePoTool::INVALID_UID
        if (isScanStarted(zoomInfo) == false && (handRightCurrentPosition.Y < spineMidPos.Y || handLeftCurrentPosition.Y < spineMidPos.Y)) {
            resetInfo(zoomInfo);
            return GePoTool::INVALID_UID;
        }

        if (handRightState == HandState_Open || handLeftState == HandState_Open) {
            resetInfo(zoomInfo);
        }
        //If the scan hasn't started, the hands are closed and above the spineMidPos, start for scanning
        else if (isScanStarted(zoomInfo) == false && isHandsClosed(handRightState, handLeftState) && isDoneGesture(zoomInfo) == false) {
            zoomLeftInfo.startPos = handLeftCurrentPosition;
            zoomLeftInfo.scanStarted = true;
            zoomRightInfo.startPos = handRightCurrentPosition;
            zoomRightInfo.scanStarted = true;
        }
        //If the scan started and the hands are still closed, keep scanning
        else if (isScanStarted(zoomInfo) && isHandsClosed(handRightState, handLeftState) && isDoneGesture(zoomInfo) == false) {
            zoomLeftInfo.timeAccumulator += delta;
            zoomRightInfo.timeAccumulator += delta;
            if (isTimeBetweenMinAndMax(zoomInfo)) {
                bool isSuccess = false;
                if (zoomType == ZOOM_TYPE::PINCH_ZOOM_IN) {
                    /* If the zoom type is pinch zoom in, left hand should be on the left side of the left hand start position and the right hand should be
                       on the right side of the right hand start position.*/
                    const float leftDistance = std::abs(zoomLeftInfo.startPos.X - handLeftCurrentPosition.X);
                    const float rightDistance = std::abs(zoomRightInfo.startPos.X - handRightCurrentPosition.X);
                    isSuccess = (zoomLeftInfo.startPos.X > handLeftCurrentPosition.X && leftDistance >= shoulderLength / 2)
                                && zoomRightInfo.startPos.X < handRightCurrentPosition.X && rightDistance >= shoulderLength / 2;
                }
                else if (zoomType == ZOOM_TYPE::PINCH_ZOOM_OUT) {
                    /* If the zoom type is pinch zoom in, left hand should be on the right side of the start position and the right hand should be on the left
                       side of the right hand start position.*/
                    const float leftDistance = std::abs(zoomLeftInfo.startPos.X - handLeftCurrentPosition.X);
                    const float rightDistance = std::abs(zoomRightInfo.startPos.X - handRightCurrentPosition.X);
                    isSuccess = (zoomLeftInfo.startPos.X < handLeftCurrentPosition.X && leftDistance >= shoulderLength / 2)
                                && zoomRightInfo.startPos.X > handRightCurrentPosition.X && rightDistance >= shoulderLength / 2;
                }
                //If the positions are in reasonable places, set the gesture as done but do not start scanning for new gesture until the hands are released and the info is reset.
                if (isSuccess) {
                    zoomLeftInfo.isDoneGesture = true;
                    zoomLeftInfo.timeAccumulator = 0;

                    zoomRightInfo.isDoneGesture = true;
                    zoomRightInfo.timeAccumulator = 0;
                    return uid;
                }
            }
            //If the timeAccumulator reaches the max time, reset info
            else if (isTimeAbovedMax(zoomInfo)) {
                resetInfo(zoomInfo);
                return GePoTool::INVALID_UID;
            }
        }
        //After the gesture is done, If the hand is released and returned to open state, end the current gesture session and reset info for new scanning.
        else if ((zoomLeftInfo.timeAccumulator == 0 && zoomRightInfo.timeAccumulator == 0) && isDoneGesture(zoomInfo) && isHandsOpen(handRightState, handLeftState)) {
            resetInfo(zoomInfo);
            return GePoTool::INVALID_UID;
        }
    }
    return GePoTool::INVALID_UID;
}

//Detect zoom in
DHandZoomIn::DHandZoomIn(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DHandZoomIn", customID)
    , m_PostureTool(postureTool)
    , m_HandPinchZoomInfoPOne()
{
}

UID DHandZoomIn::detect(IBody *body, const float &delta)
{
    return HandZoomHelper::detectHandZoom(m_PostureTool, body, ZOOM_TYPE::ZOOM_IN, m_HandPinchZoomInfoPOne, delta, getID());
}

//Detect zoom out
DHandZoomOut::DHandZoomOut(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DHandZoomOut", customID)
    , m_PostureTool(postureTool)
    , m_HandPinchZoomInfoPOne()
{
}

UID DHandZoomOut::detect(IBody *body, const float &delta)
{
    return HandZoomHelper::detectHandZoom(m_PostureTool, body, ZOOM_TYPE::ZOOM_OUT, m_HandPinchZoomInfoPOne, delta, getID());
}

//Detect pinch zoom out
DHandPinchZoomOut::DHandPinchZoomOut(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DHandPinchZoomOut", customID)
    , m_PostureTool(postureTool)
    , m_HandPinchZoomInfo(std::make_pair(HandZoomInfo(), HandZoomInfo()))
{
}

UID DHandPinchZoomOut::detect(IBody *body, const float &delta)
{
    return HandZoomHelper::detectHandPinchZoom(m_PostureTool, body, ZOOM_TYPE::PINCH_ZOOM_OUT, m_HandPinchZoomInfo, delta, getID());
}

//Detect pinch zoom in
DHandPinchZoomIn::DHandPinchZoomIn(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DHandPinchZoomIn", customID)
    , m_PostureTool(postureTool)
    , m_HandPinchZoomInfo(std::make_pair(HandZoomInfo(), HandZoomInfo()))
{
}

UID DHandPinchZoomIn::detect(IBody *body, const float &delta)
{
    return HandZoomHelper::detectHandPinchZoom(m_PostureTool, body, ZOOM_TYPE::PINCH_ZOOM_IN, m_HandPinchZoomInfo, delta, getID());
}

