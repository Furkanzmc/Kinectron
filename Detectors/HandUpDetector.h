#ifndef HANDUPDETECTOR_H
#define HANDUPDETECTOR_H
#include "GestureDetectorBase.h"
class GePoTool;

class DHandUpLeft : public DGestureBase
{
public:
    DHandUpLeft(GePoTool &postureTool, const UID &customID = 0);

    UID detect(IBody *body, const float &delta = 0) override;

private:
    GePoTool &m_PostureTool;
};

class DHandUpRight : public DGestureBase
{
public:
    DHandUpRight(GePoTool &postureTool, const UID &customID = 0);

    UID detect(IBody *body, const float &delta = 0) override;

private:
    GePoTool &m_PostureTool;

};

#endif // HANDUPDETECTOR_H
