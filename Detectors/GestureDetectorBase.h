#ifndef GESTUREDETECTORBASE
#define GESTUREDETECTORBASE
#include "Kinect.h"
#include <string>
using UID = int;//Short for Unique ID

/**
 * @brief The DGestureBase class
 * When naming the classes that inherit DGestureBase, put a "D" suffix to indicate that the class is a Detector
 */
class DGestureBase
{
public:
    enum class DETECTION_TYPE {
        GESTURE,
        POSTURE
    };

    static const int INVALID_BODY_INDEX = -1;
    static const int INVALID_TAG = -1;

public:
    const UID &getUniqueID() const
    {
        return m_UID;
    }

    const UID &getTag() const
    {
        return m_Tag;
    }

    /**
     * @brief Returns the active ID. If the tag is valid, returns the tag. Otherwise returns UID.
     * @todo Find a better name for this
     * @return
     */
    const UID &getID() const
    {
        return m_Tag > INVALID_TAG ? m_Tag : m_UID;
    }

    const int &getBodyIndex() const
    {
        return m_BodyIndex;
    }

    void setBodyIndex(const unsigned int &bodyIndex)
    {
        if (bodyIndex >= BODY_COUNT) {
            return;
        }

        m_BodyIndex = bodyIndex;
    }

    const DETECTION_TYPE &getType() const
    {
        return m_Type;
    }

    void setDescription(const std::string &description)
    {
        m_Description = description;
    }

    const std::string &getDescription() const
    {
        return m_Description;
    }

    /**
     * @brief This function is called by the GePoTool::determinePlayerPosture method. If the detection is successful, this function should return the
     * unique ID of the detector. If the detector has a custom ID that is equal to/above 1000, then custom ID should be returned.
     * @param body
     * @param delta
     * @return
     */
    virtual UID detect(IBody *body, const float &delta) = 0;

protected:
    const UID m_UID;
    const DETECTION_TYPE m_Type;
    const UID m_Tag;
    std::string m_Description;
    int m_BodyIndex;//The intial value is -1, so that a check can be done to see if it's been set or not

protected:
    DGestureBase(const UID &gestureUID, const DETECTION_TYPE &type, const std::string &description, const UID &tag = INVALID_TAG)
        : m_UID(gestureUID)
        , m_Type(type)
        , m_Tag(tag)
        , m_Description(description)
        , m_BodyIndex(INVALID_BODY_INDEX)
    {}

    DGestureBase(const UID &gestureUID, const DETECTION_TYPE &type, const UID &tag = INVALID_TAG)
        : m_UID(gestureUID)
        , m_Type(type)
        , m_Tag(tag)
        , m_Description("")
        , m_BodyIndex(INVALID_BODY_INDEX)
    {}

private:
    DGestureBase(const DGestureBase &base) = delete;
};

#endif // GESTUREDETECTORBASE

