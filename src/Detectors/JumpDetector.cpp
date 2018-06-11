#include "Kinectron/Detectors/JumpDetector.h"
// Local
#include "Kinectron/GePoTool.h"
#include "Kinectron/KinectHandler.h"

DJump::DJump(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DJump", customID)
    , m_PostureTool(postureTool)
    , m_TimeAccumulatorJump(0)
    , m_DidJump(false)
{

}

UID DJump::detect(IBody *body, const float &delta)
{
    if (!body) {
        return GePoTool::INVALID_UID;
    }

    float &timeAccumulator = m_TimeAccumulatorJump;
    bool &didJump = m_DidJump;
    const float requiredMaxTimeForJump = .3f;
    const float requiredMinTimeForJump = .15f;

    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);
    if (SUCCEEDED(hr)) {
        const CameraSpacePoint rightFootPos = joints[JointType_AnkleRight].Position;
        const CameraSpacePoint leftFootPos = joints[JointType_AnkleLeft].Position;
        const CameraSpacePoint leftKneePos = joints[JointType_KneeLeft].Position;
        const CameraSpacePoint rightKneePos = joints[JointType_KneeRight].Position;
        const float kneeToFootDistance = max(std::abs(leftKneePos.Y - leftFootPos.Y), std::abs(rightKneePos.Y - rightFootPos.Y));
        const float jumpDistance = kneeToFootDistance * 0.4f;

        const double rightFootDist = m_PostureTool.getKinectHandler().getDistanceFromFloor(rightFootPos);
        const double leftFootDist = m_PostureTool.getKinectHandler().getDistanceFromFloor(leftFootPos);

        const float feetLenght = std::abs(rightFootPos.X - leftFootPos.X);
        const float feetLenghtOnZ = std::abs(rightFootPos.Z - leftFootPos.Z);

        //If the feet are apart more than 1.5 times of the shoulder lenght, return false
        if (feetLenght > kneeToFootDistance * 1.5f) {
            timeAccumulator = 0;
            didJump = false;
            return GePoTool::INVALID_UID;
        }
        //If the feet are apart from each other on the Z-axis more than the half of the shoulder legnth, return false
        if (feetLenghtOnZ > kneeToFootDistance * .5f) {
            timeAccumulator = 0;
            didJump = false;
            return GePoTool::INVALID_UID;
        }

        if (didJump == false) {
            timeAccumulator += delta;

            if (timeAccumulator >= requiredMinTimeForJump && timeAccumulator <= requiredMaxTimeForJump) {
                if (rightFootDist >= jumpDistance && leftFootDist >= jumpDistance) {
                    didJump = true;
                    timeAccumulator = 0;
                    return getID();
                }
            }
            else if (timeAccumulator > requiredMaxTimeForJump) {
                timeAccumulator = 0;
                didJump = false;
            }
        }
        else if (timeAccumulator == 0 && rightFootDist <= jumpDistance && leftFootDist <= jumpDistance) {
            timeAccumulator = 0;
            didJump = false;
        }
    }
    return GePoTool::INVALID_UID;
}
