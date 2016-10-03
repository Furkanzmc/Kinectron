#include "TomDetector.h"
// Local
#include "GePoTool.h"

DTom::DTom(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DTom", customID)
    , m_PostureTool(postureTool)
{

}

UID DTom::detect(IBody *body, const float &deltaTime)
{
    if (!body) {
        return GePoTool::INVALID_UID;
    }

    bool isTomPosture = false;
    Joint joints[JointType_Count];
    HRESULT hr = body->GetJoints(_countof(joints), joints);

    if (SUCCEEDED(hr)) {
        //If hand , elbow and shoulder aligns together on the X axis then the arm is open
        const CameraSpacePoint handRightPos = joints[JointType_HandRight].Position;
        const CameraSpacePoint elbowRightPos = joints[JointType_ElbowRight].Position;
        const CameraSpacePoint shoulderRightPos = joints[JointType_ShoulderRight].Position;

        const CameraSpacePoint handLeftPos = joints[JointType_HandLeft].Position;
        const CameraSpacePoint elbowLeftPos = joints[JointType_ElbowLeft].Position;
        const CameraSpacePoint shoulderLeftPos = joints[JointType_ShoulderLeft].Position;

        if (std::abs(handRightPos.Y) > std::abs(elbowRightPos.Y)) {
            return GePoTool::INVALID_UID;
        }
        if (std::abs(handLeftPos.Y) > std::abs(elbowLeftPos.Y)) {
            return GePoTool::INVALID_UID;
        }

        if (elbowRightPos.Y - shoulderRightPos.Y > .15f) {
            return GePoTool::INVALID_UID;
        }
        if (elbowRightPos.Y - shoulderRightPos.Y < -.15f) {
            return GePoTool::INVALID_UID;
        }
        if (elbowLeftPos.Y - shoulderLeftPos.Y > .15f) {
            return GePoTool::INVALID_UID;
        }
        if (elbowLeftPos.Y - shoulderLeftPos.Y < -.15f) {
            return GePoTool::INVALID_UID;
        }

        float rightForearmLength = std::sqrtf(std::powf((handRightPos.X - elbowRightPos.X), 2) + std::powf((handRightPos.Y - elbowRightPos.Y), 2));
        CameraSpacePoint bottomPos;
        bottomPos.X = elbowRightPos.X;
        bottomPos.Y = handRightPos.Y;
        bottomPos.Z = handRightPos.Z;
        float bottomLenght = std::sqrtf(std::powf((bottomPos.X - elbowRightPos.X), 2) + std::powf((bottomPos.Y - elbowRightPos.Y), 2));
        float rightAngle = m_PostureTool.toDegree(std::acosf(bottomLenght / rightForearmLength));

        float leftForearmLength = std::sqrtf(std::powf((handLeftPos.X - elbowLeftPos.X), 2) + std::powf((handLeftPos.Y - elbowLeftPos.Y), 2));
        bottomPos;
        bottomPos.X = elbowLeftPos.X;
        bottomPos.Y = handLeftPos.Y;
        bottomPos.Z = handLeftPos.Z;
        bottomLenght = std::sqrtf(std::powf((bottomPos.X - elbowLeftPos.X), 2) + std::powf((bottomPos.Y - elbowLeftPos.Y), 2));
        float leftAngle = m_PostureTool.toDegree(std::acosf(bottomLenght / leftForearmLength));

        if ((rightAngle > 5 && rightAngle < 50) && (leftAngle > 5 && leftAngle < 50)) {
            isTomPosture = true;
        }
    }

    return isTomPosture ? getID() : GePoTool::INVALID_UID;
}
