#ifndef SQUATDETECTOR_H
#define SQUATDETECTOR_H
#include "GestureDetectorBase.h"
class GePoTool;

class DSquat : public DGestureBase
{
public:
    DSquat(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);
    UID detect(IBody *body, const float &delta) override;

private:
    GePoTool &m_PostureTool;
};

#endif // SQUATDETECTOR_H
