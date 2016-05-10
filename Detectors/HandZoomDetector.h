#ifndef HANDZOOMDETECTOR_H
#define HANDZOOMDETECTOR_H
#include "GestureDetectorBase.h"
class GePoTool;

class HandZoomHelper
{
public:
    struct HandZoomInfo {
        float timeAccumulator = 0;
        const float minimalTime = .3f, maximumTime = .9f;
        CameraSpacePoint startPos;
        bool scanStarted = false, isDoneGesture = false;
    };

    enum class ZOOM_TYPE {
        ZOOM_IN,//Make fist with one hand and pull
        ZOOM_OUT,//Make fist with one hand  and push
        PINCH_ZOOM_IN,//Make fist with both hands and pan out
        PINCH_ZOOM_OUT//Make fist with both hands and pull them together
    };

    using PinchZoomInfo = std::pair<HandZoomInfo, HandZoomInfo>;

public:
    static UID detectHandZoom(GePoTool &postureTool, IBody *body, const ZOOM_TYPE &zoomType, HandZoomInfo &zoomInfo, const float &delta, const UID &uid);
    static UID detectHandPinchZoom(GePoTool &postureTool, IBody *body, const ZOOM_TYPE &zoomType, PinchZoomInfo &zoomInfo, const float &delta, const UID &uid);
};

//Detect Zoom In
class DHandZoomIn : public DGestureBase
{
public:
    DHandZoomIn(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);
    UID detect(IBody *body, const float &delta) override;

private:
    using HandZoomInfo = HandZoomHelper::HandZoomInfo;
    using ZOOM_TYPE = HandZoomHelper::ZOOM_TYPE;
    GePoTool &m_PostureTool;
    HandZoomInfo m_HandPinchZoomInfoPOne;
};

//Detect Zoom Out
class DHandZoomOut : public DGestureBase
{
public:
    DHandZoomOut(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);
    UID detect(IBody *body, const float &delta) override;

private:
    using HandZoomInfo = HandZoomHelper::HandZoomInfo;
    using ZOOM_TYPE = HandZoomHelper::ZOOM_TYPE;
    GePoTool &m_PostureTool;
    HandZoomInfo m_HandPinchZoomInfoPOne;
};

//Detect Pinch Zoom Out
class DHandPinchZoomOut : public DGestureBase
{
public:
    DHandPinchZoomOut(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);
    UID detect(IBody *body, const float &delta) override;

private:
    using HandZoomInfo = HandZoomHelper::HandZoomInfo;
    using ZOOM_TYPE = HandZoomHelper::ZOOM_TYPE;
    GePoTool &m_PostureTool;
    HandZoomHelper::PinchZoomInfo m_HandPinchZoomInfo;
};

//Detect Pinch Zoom In
class DHandPinchZoomIn : public DGestureBase
{
public:
    DHandPinchZoomIn(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);
    UID detect(IBody *body, const float &delta) override;

private:
    using HandZoomInfo = HandZoomHelper::HandZoomInfo;
    using ZOOM_TYPE = HandZoomHelper::ZOOM_TYPE;
    GePoTool &m_PostureTool;
    HandZoomHelper::PinchZoomInfo m_HandPinchZoomInfo;
};

#endif // HANDZOOMDETECTOR_H
