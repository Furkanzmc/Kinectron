#ifndef TOMDETECTOR_H
#define TOMDETECTOR_H
#include "GestureDetectorBase.h"
class GePoTool;

class DTom : public DGestureBase
{
public:
    DTom(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);

    UID detect(IBody *body, const float &deltaTime) override;

private:
    GePoTool &m_PostureTool;
};

#endif // TOMDETECTOR_H
