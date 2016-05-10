#ifndef BRUSHDETECTOR_H
#define BRUSHDETECTOR_H
#include "GestureDetectorBase.h"
class GePoTool;

/**
 * @brief The BrushDetector class - This is the gesture that you put your hand in front of your mouth and start waving your hand up and down.
 */
class DBrush : public DGestureBase
{
public:
    DBrush(GePoTool &postureTool, const UID &customID = DGestureBase::INVALID_TAG);

    UID detect(IBody *body, const float &delta) override;

private:
    struct BrushInfo {
        float timeAccumulator = 0;
        const float minimalTime = .07f, maximumTime = .3f;
        bool isHandAboveHead = false;
    };

    BrushInfo m_BrushInfo;
};

#endif // BRUSHDETECTOR_H
