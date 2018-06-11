#pragma once
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
