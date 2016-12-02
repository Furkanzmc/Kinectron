#include "Kinectron/Detectors/BrushDetector.h"
// Local
#include "Kinectron/GePoTool.h"

DBrush::DBrush(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DBrush", customID)
    , m_BrushInfo()
{

}

UID DBrush::detect(IBody *body, const float &delta)
{
    if (!body) {
        return GePoTool::INVALID_UID;
    }

    auto resetInfo = [](BrushInfo & info) {
        info.isHandAboveHead = false;
        info.timeAccumulator = 0;
    };
    BrushInfo &brushInfo = m_BrushInfo;

    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        CameraSpacePoint handPosition = {0};
        const CameraSpacePoint spineShoulderPos = joints[JointType_SpineShoulder].Position;
        const CameraSpacePoint headPos = joints[JointType_Head].Position;

        //Set the higher hand as the active one
        if (joints[JointType_HandLeft].Position.Y > joints[JointType_HandRight].Position.Y) {
            handPosition = joints[JointType_HandLeft].Position;
        }
        else {
            handPosition = joints[JointType_HandRight].Position;
        }

        //If the hand is below the neck return GePoTool::INVALID_UID
        if (handPosition.Y < spineShoulderPos.Y) {
            resetInfo(brushInfo);
            return GePoTool::INVALID_UID;
        }

        if (brushInfo.isHandAboveHead == false && handPosition.Y > headPos.Y) {
            brushInfo.isHandAboveHead = true;
        }
        else if (brushInfo.isHandAboveHead) {
            brushInfo.timeAccumulator += delta;
            if (brushInfo.timeAccumulator >= brushInfo.minimalTime && brushInfo.timeAccumulator <= brushInfo.maximumTime) {
                if (handPosition.Y < headPos.Y) {
                    resetInfo(brushInfo);
                    return getID();
                }
            }
            else if (brushInfo.timeAccumulator > brushInfo.maximumTime) {
                resetInfo(brushInfo);
                return GePoTool::INVALID_UID;
            }
        }
    }
    return GePoTool::INVALID_UID;
}
