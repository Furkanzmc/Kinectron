#pragma once
#include "GestureDetectorBase.h"
class GePoTool;

class DSwipeLeft : public DGestureBase
{
public:
    DSwipeLeft(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);

    UID detect(IBody *body, const float &delta) override;

private:
    GePoTool &m_PostureTool;

    // --- For Swipe Gestures ---// 
    const float m_MinimalTimeForSwipe, m_MaximumTimeForSwipe;

    //Swipe Left
    float m_TimeAccumulatorSwipe;
    bool m_IsHandRightOnRight;
};

class DSwipeRight : public DGestureBase
{
public:
    DSwipeRight(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);

    UID detect(IBody *body, const float &delta) override;

private:
    GePoTool &m_PostureTool;

    // --- For Swipe Gestures ---// 
    const float m_MinimalTimeForSwipe, m_MaximumTimeForSwipe;

    //Swipe Right
    float m_TimeAccumulatorSwipe;
    bool m_IsHandLeftOnLeft;
};
