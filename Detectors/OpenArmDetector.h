#ifndef OPENARMDETECTOR_H
#define OPENARMDETECTOR_H
#include "GestureDetectorBase.h"
class GePoTool;

class DOpenArmLeft : public DGestureBase
{
public:
    DOpenArmLeft(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);

    UID detect(IBody *body, const float &delta = 0);
    UID detectOpenRightArm(IBody *body, const float &delta = 0);

private:
    GePoTool &m_PostureTool;
    float m_Duration;
    const float m_DurationLimit;
    bool m_IsGestureInitiated;

private:
    void resetGesture();
};

class DOpenArmRight : public DGestureBase
{
public:
    DOpenArmRight(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);

    UID detect(IBody *body, const float &delta = 0);

private:
    GePoTool &m_PostureTool;
    float m_Duration;
    const float m_DurationLimit;
    bool m_IsGestureInitiated;

private:
    void resetGesture();
};

class DOpenArms : public DGestureBase
{
public:
    DOpenArms(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);

    UID detect(IBody *body, const float &delta = 0);

private:
    GePoTool &m_PostureTool;
    float m_Duration;
    const float m_DurationLimit;
    bool m_IsGestureInitiated;

private:
    bool detectOpenRightArm(IBody *body);
    bool detectOpenLeftArm(IBody *body);
    void resetGesture();
};

#endif // OPENARMDETECTOR_H
