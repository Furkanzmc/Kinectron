#include "SkeletonSmoother.h"
// Local
#include "GePoTool.h"
#include "KinectHandler.h"
// Submodules
#include "Ceset.h"

SkeletonSmoother::SkeletonSmoother(ICoordinateMapper *coordinateMapper)
    : m_CoordinateMapper(coordinateMapper)
    , m_PostureTool(nullptr)
    , m_SmoothScale(1.f)
{
    setupSmoother();
    std::fill(m_BodyTrackedStatuses.begin(), m_BodyTrackedStatuses.end(), false);
}

SkeletonSmoother::SkeletonSmoother(GePoTool *postureTool)
    : m_PostureTool(postureTool)
    , m_CoordinateMapper(nullptr)
    , m_SmoothScale(1.f)
{
    if (postureTool) {
        m_CoordinateMapper = m_PostureTool->getKinectHandler().getCoordinateMapper();
    }

    setupSmoother();
    std::fill(m_BodyTrackedStatuses.begin(), m_BodyTrackedStatuses.end(), false);
}

void SkeletonSmoother::update(const float &delta)
{
    if (!m_PostureTool) {
        LOG_WARNING("m_PostureTool is nullptr. Try using SkeletonSmoother::updateJointPositions.");
        return;
    }

    // Reset the body properties that is bigger than the detected body count
    for (std::size_t index = m_PostureTool->getDetectedBodyCount(); index < BODY_COUNT; index++) {
        reset(index);
    }

    for (int bodyIndex = 0; bodyIndex < BODY_COUNT; bodyIndex++) {
        IBody *body = m_PostureTool->getBody(bodyIndex);
        if (!body) {
            reset(bodyIndex);
            return;
        }

        Joint joints[JointType_Count];
        HRESULT hr = body->GetJoints(_countof(joints), joints);
        if (SUCCEEDED(hr)) {
            updateJointPositions(bodyIndex, delta, joints);
        }
        else {
            reset(bodyIndex);
        }
    }
}

void SkeletonSmoother::updateJointPositions(const unsigned int &bodyIndex, const float &delta, Joint *joints)
{
    if (joints == nullptr || bodyIndex >= BODY_COUNT) {
        return;
    }

    m_BodyTrackedStatuses[bodyIndex] = true;
    for (unsigned int jointIndex = 0; jointIndex < JointType_Count; jointIndex++) {
        const PointF screenPos = mapBodyPointToScreenPoint(joints[jointIndex].Position);
        JointArray &jointPositions = m_JointPositions.at(bodyIndex);
        JointProp &prop = jointPositions.at(jointIndex);
        prop.spacePoint = joints[jointIndex].Position;
        prop.type = static_cast<JointType>(jointIndex);

        if (pointEquals(prop.pos, pointZero()) || prop.isDirty) {
            prop.pos.X = screenPos.X * m_PositionScale.X;
            prop.pos.Y = screenPos.Y * m_PositionScale.Y;
            prop.isDirty = false;
        }

        prop.attractionPoint.X = screenPos.X * m_PositionScale.X;
        prop.attractionPoint.Y = screenPos.Y * m_PositionScale.Y;
    }

    for (unsigned int jointIndex = 0; jointIndex < m_JointPositions.at(bodyIndex).size(); jointIndex++) {
        m_JointPositions.at(bodyIndex).at(jointIndex).updatePos(delta, m_SmoothScale);
    }
}

void SkeletonSmoother::reset(const unsigned int &bodyIndex)
{
    m_BodyTrackedStatuses[bodyIndex] = false;
    for (unsigned int i = 0; i < JointType_Count; i++) {
        m_JointPositions.at(bodyIndex).at(i).reset();
    }
}

void SkeletonSmoother::setSmoothScale(float scale)
{
    if (scale <= 0) {
        return;
    }

    m_SmoothScale = scale;
}

float SkeletonSmoother::getSmoothScale() const
{
    return m_SmoothScale;
}

void SkeletonSmoother::setPositionScale(const PointF &scale)
{
    if (scale.X <= 0 || scale.Y <= 0) {
        return;
    }

    m_PositionScale = scale;
    for (unsigned int bodyIndex = 0; bodyIndex < m_JointPositions.size(); bodyIndex++) {
        for (unsigned int i = 0; i < JointType_Count; i++) {
            m_JointPositions.at(bodyIndex).at(i).isDirty = true;
        }
    }
}

PointF SkeletonSmoother::getPositionScale() const
{
    return m_PositionScale;
}

const std::array<SkeletonSmoother::JointProp, JointType_Count> &SkeletonSmoother::getJointProperties(const unsigned int &bodyIndex) const
{
    return m_JointPositions.at(bodyIndex);
}

PointF SkeletonSmoother::getJointPosition(const unsigned int &bodyIndex, const unsigned int &type) const
{
    return m_JointPositions.at(bodyIndex).at(type).pos;
}

void SkeletonSmoother::enableJointDrawing(unsigned int bodyIndex, JointType jointType, bool enable)
{
    m_JointPositions.at(bodyIndex).at(jointType).isDraw = enable;
}

bool SkeletonSmoother::isJointDrew(unsigned int bodyIndex, JointType jointType) const
{
    return m_JointPositions.at(bodyIndex).at(jointType).isDraw;
}

bool SkeletonSmoother::isBodyTracked(const unsigned int &bodyIndex) const
{
    if (bodyIndex >= BODY_COUNT) {
        return false;
    }

    return m_BodyTrackedStatuses.at(bodyIndex);
}

void SkeletonSmoother::setupSmoother()
{
    m_PositionScale.X = 1.f;
    m_PositionScale.Y = 1.f;

    std::fill(m_JointPositions.at(0).begin(), m_JointPositions.at(0).end(), JointProp());
    std::fill(m_JointPositions.at(1).begin(), m_JointPositions.at(1).end(), JointProp());
    std::fill(m_JointPositions.at(2).begin(), m_JointPositions.at(2).end(), JointProp());
    std::fill(m_JointPositions.at(3).begin(), m_JointPositions.at(3).end(), JointProp());
    std::fill(m_JointPositions.at(4).begin(), m_JointPositions.at(4).end(), JointProp());
    std::fill(m_JointPositions.at(5).begin(), m_JointPositions.at(5).end(), JointProp());
}

PointF SkeletonSmoother::mapBodyPointToScreenPoint(const CameraSpacePoint &bodyPoint)
{
    // Calculate the body's position on the screen
    PointF screenPos = {0, 0};
    ColorSpacePoint colorPoint = {0, 0};

    if (m_CoordinateMapper) {
        m_CoordinateMapper->MapCameraPointToColorSpace(bodyPoint, &colorPoint);
        screenPos.X = colorPoint.X;
        screenPos.Y = colorPoint.Y;
    }

    return screenPos;
}

bool SkeletonSmoother::pointEquals(const PointF &p1, const PointF &p2) const
{
    return (p1.X == p2.X) && (p1.Y == p2.Y);
}

PointF SkeletonSmoother::pointZero() const
{
    PointF p = {0, 0};
    return p;
}
