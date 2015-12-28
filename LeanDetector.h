#ifndef LEANDETECTOR_H
#define LEANDETECTOR_H
#include "GestureDetectorBase.h"
class GePoTool;

class DLeanLeft : public DGestureBase
{
public:
    DLeanLeft(GePoTool &postureTool, const UID &customID = 0);

    UID detect(IBody *body, const float &delta = 0) override;

private:
    GePoTool &m_PostureTool;
};

class DLeanRight : public DGestureBase
{
public:
    DLeanRight(GePoTool &postureTool, const UID &customID = 0);

    UID detect(IBody *body, const float &delta = 0) override;

private:
    GePoTool &m_PostureTool;
};

#endif // LEANDETECTOR_H
