#include "LeanDetector.h"
// Local
#include "GePoTool.h"

DLeanLeft::DLeanLeft(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DLeanLeft", customID)
    , m_PostureTool(postureTool)
{
}

UID DLeanLeft::detect(IBody *body, const float &delta)
{
    const PointF leanAmount = m_PostureTool.getLeanAmount(body);
    return leanAmount.X < -.6f == true ? getID() : GePoTool::INVALID_UID;
}


DLeanRight::DLeanRight(GePoTool &postureTool, const UID &customID)
    : DGestureBase(postureTool.getUID(), DETECTION_TYPE::GESTURE, "DLeanRight", customID)
    , m_PostureTool(postureTool)
{
}

UID DLeanRight::detect(IBody *body, const float &delta)
{
    const PointF leanAmount = m_PostureTool.getLeanAmount(body);
    return leanAmount.X >= .6f == true ? getID() : GePoTool::INVALID_UID;
}
