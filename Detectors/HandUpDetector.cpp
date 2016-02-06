#include "HandUpDetector.h"
#include "GePoTool.h"

DHandUpLeft::DHandUpLeft(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DHandUpLeft", customID)
    , m_PostureTool(postureTool)
{
}

UID DHandUpLeft::detect(IBody *body, const float &delta)
{
    if (!body) {
        return GePoTool::INVALID_UID;
    }

    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);

    if (SUCCEEDED(hr)) {
        const CameraSpacePoint handPos = joints[JointType_HandLeft].Position;
        const CameraSpacePoint headPos = joints[JointType_Head].Position;

        //hand is up
        if (handPos.Y >= headPos.Y - .005f) {
            return getID();
        }
    }
    return GePoTool::INVALID_UID;;
}

//Right hand up

DHandUpRight::DHandUpRight(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DHandUpRight", customID)
    , m_PostureTool(postureTool)
{
}

UID DHandUpRight::detect(IBody *body, const float &delta)
{
    if (!body) {
        return GePoTool::INVALID_UID;
    }

    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);

    if (SUCCEEDED(hr)) {
        const CameraSpacePoint handPos = joints[JointType_HandRight].Position;
        const CameraSpacePoint headPos = joints[JointType_Head].Position;

        //The hand is up
        if (handPos.Y >= headPos.Y - .005f) {
            return getID();
        }
    }
    return GePoTool::INVALID_UID;
}
