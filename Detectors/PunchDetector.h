#ifndef PUNCHDETECTOR_H
#define PUNCHDETECTOR_H
#include "GestureDetectorBase.h"
class GePoTool;

class DPunchRight : public DGestureBase
{
public:
    DPunchRight(GePoTool &postureTool, const UID &customID = 0);

    UID detect(IBody *body, const float &delta) override;
    UID detectLeftPunch(IBody *body, const float &delta);

private:
    GePoTool &m_PostureTool;
    //Punch related
    float m_TimeAccumulatorRightPunch, m_TimeAccumulatorLeftPunch;
    CameraSpacePoint m_TempPosRightPunch, m_TempPosLeftPunch;
    bool m_DidLeftPunchFront, m_DidRightPunchFront;
};

class DPunchLeft : public DGestureBase
{
public:
    DPunchLeft(GePoTool &postureTool, const UID &customID = 0);

    UID detect(IBody *body, const float &delta) override;

private:
    GePoTool &m_PostureTool;
    //Punch related
    float m_TimeAccumulatorRightPunch, m_TimeAccumulatorLeftPunch;
    CameraSpacePoint m_TempPosRightPunch, m_TempPosLeftPunch;
    bool m_DidLeftPunchFront, m_DidRightPunchFront;
};
#endif // PUNCHDETECTOR_H