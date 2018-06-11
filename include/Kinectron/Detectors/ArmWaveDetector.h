#pragma once
#include "GestureDetectorBase.h"
class GePoTool;

class ArmWaveDetectorHelper
{
public:
    struct ArmWaveInfo {
        const float minimalTime = .2f, maximumTime = .5f;
        float timeAccumulator = 0;

        bool isHandAboveHead = false,
             isHandAboveShoulder = false,
             isElbowAboveShoulder = false,
             isElbowAboveHead = false,
             isGestureDone = false;
    };

    enum class GESTURE_SIDE {
        LEFT,
        RIGHT
    };

public:
    static UID detectArmWaveGesture(IBody *body, const GESTURE_SIDE &gestureSide, ArmWaveInfo &waveInfo, const float &delta, const UID &uid);
};

class DArmWaveLeft : public DGestureBase
{
public:
    DArmWaveLeft(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);

    UID detect(IBody *body, const float &delta) override;

private:
    using ArmWaveInfo = ArmWaveDetectorHelper::ArmWaveInfo;
    using GESTURE_SIDE = ArmWaveDetectorHelper::GESTURE_SIDE;

    GePoTool &m_PostureTool;
    ArmWaveInfo m_ArmWaveInfo;
};

class DArmWaveRight : public DGestureBase
{
public:
    DArmWaveRight(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);

    UID detect(IBody *body, const float &delta) override;

private:
    using ArmWaveInfo = ArmWaveDetectorHelper::ArmWaveInfo;
    using GESTURE_SIDE = ArmWaveDetectorHelper::GESTURE_SIDE;

    GePoTool &m_PostureTool;
    ArmWaveInfo m_ArmWaveInfo;
};
