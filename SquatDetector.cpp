#include "SquatDetector.h"
#include "GePoTool.h"

DSquat::DSquat(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DSquat", customID)
    , m_PostureTool(postureTool)
{

}

UID DSquat::detect(IBody *body, const float &delta)
{
    if (!body) {
        return GePoTool::INVALID_UID;
    }

    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        const CameraSpacePoint rightHipPos = joints[JointType_HipRight].Position;
        const CameraSpacePoint rightFootPos = joints[JointType_FootLeft].Position;
        const CameraSpacePoint leftFootPos = joints[JointType_FootRight].Position;
        const float shoulderLenght = std::abs(joints[JointType_ShoulderLeft].Position.X - joints[JointType_ShoulderRight].Position.X);
        const float feetLenght = std::abs(rightFootPos.X - leftFootPos.X);
        const float feetLenghtOnZ = std::abs(rightFootPos.Z - leftFootPos.Z);

        if (m_PostureTool.getKinectHandler().getDistanceFromFloor(rightHipPos) <= shoulderLenght * 2.f) {
            return getID();
        }

        //If the feet are apart more than 1.5 times of the shoulder lenght, return false
        if (feetLenght > shoulderLenght * 1.5f) {
            return GePoTool::INVALID_UID;
        }
        //If the feet are apart from each other on the Z-axis more than the half of the shoulder legnth, return false
        if (feetLenghtOnZ > shoulderLenght * .5f) {
            return GePoTool::INVALID_UID;
        }
    }

    return GePoTool::INVALID_UID;
}
