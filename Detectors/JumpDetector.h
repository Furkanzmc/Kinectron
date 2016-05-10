#ifndef JUMPDETECTOR_H
#define JUMPDETECTOR_H
#include "GestureDetectorBase.h"
class GePoTool;

class DJump : public DGestureBase
{
public:
    DJump(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);

    UID detect(IBody *body, const float &delta) override;

private:
    GePoTool &m_PostureTool;
    //Jump related
    float m_TimeAccumulatorJump;
    bool m_DidJump;
};

#endif // JUMPDETECTOR_H
