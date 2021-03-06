#pragma once
#include "GestureDetectorBase.h"
class GePoTool;

class DKickRight : public DGestureBase
{
public:
    DKickRight(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);

    UID detect(IBody *body, const float &delta) override;

private:
    GePoTool &m_PostureTool;

    //Kick related
    float m_TimeAccumulatorKickLeft, m_TimeAccumulatorKickRight;
    CameraSpacePoint m_KickLeftStartPos, m_KickRightStartPos;
    bool m_DidKickLeft, m_DidKickRight;
};

class DKickLeft : public DGestureBase
{
public:
    DKickLeft(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);

    UID detect(IBody *body, const float &delta) override;

private:
    GePoTool &m_PostureTool;

    //Kick related
    float m_TimeAccumulatorKickLeft, m_TimeAccumulatorKickRight;
    CameraSpacePoint m_KickLeftStartPos, m_KickRightStartPos;
    bool m_DidKickLeft, m_DidKickRight;
};
