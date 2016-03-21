#ifndef GEPOTOOL_H
#define GEPOTOOL_H
#include "GestureDetectorBase.h"
#include <functional>
#include <vector>
#include <array>
#include <string>

using BodyIndex = unsigned int;
class KinectHandler;

class GePoTool
{
public:
    std::function<void(const BodyIndex &)> onBodyLost, onBodyFound;

    const static int INVALID_ANGLE = -720;
    const static UID INVALID_UID = 0;

    struct BodyRect {
        float x = 0, y = 0, width = 0, height = 0;
    };

    enum PLAYER : unsigned int {
        ONE = 0,
        TWO,
        THREE,
        FOUR,
        FIVE,
        SIX
    };

public:
    GePoTool(const unsigned int &sensorIniteType);
    ~GePoTool();

    /**
     * @brief UIDs start with 1, if a UID is 0 it means it's invalid. You can
     * @param customID
     * @return
     */
    UID getUID();

    DGestureBase *getDetector(const UID &detectorID);

    bool removeDetector(DGestureBase *detector);

    KinectHandler &getKinectHandler();
    IBody *getBody(const BodyIndex &player);
    IBody *getBodyWithTrackingID(const UINT64 &trackingID);

    /**
     * @brief If no body is found, returns -1
     * @param trackingID
     * @return
     */
    int getBodyIndexWithTrackingID(const UINT64 &trackingID);
    UINT64 getTrackingID(IBody *body) const;

    /**
     * @brief Returns a pair containing the bool value whether the hand is closed or not.
     * @param body
     * @return std::pair<bool, bool> - First is left hand, second is right hand
     */
    std::pair<HandState, HandState> getHandStates(IBody *body) const;

    /**
     * @brief 0 is the natural state. When X is 1 the player is leaning all the way to the right, if -1 the player is leaning all the way to the left.
     * When Y is 1 the player is leaning all the way front, if *1 the player is leaning all the way back. Leaning is
     * putting your wight on one leg. It's not just bending your torso, but your whole body.
     * @param body
     * @return
     */
    PointF getLeanAmount(IBody *body) const;

    /**
      * @brief Returns the angle between the two hands. Think of this as getting the angle between two vectors.
      * Returns minus degree if the left and is below the right.
      * @param body
      * @return If an angle cannot be calculated the return value is -720, since that is a number that can't be achieved.
      */
    float getAngleBetweenHands(IBody *body) const;

    /**
     * @brief This functions runs the detect(IBody*, const float&) function of each gesture detector and stores the detected UID. The detectors with a different body index
     * than the given one is skipped.
     * If the detector has a custom ID that is equal to/above GestureDetectorBase::CUSTOM_ID_START_VALUE, it stores the custom ID.
     * This way, you can attach custom IDs other than the unique one to match your gestures with other things.
     * Every GestureDetectorBase has a BodyIndex variable and its initial value is GestureDetectorBase::CUSTOM_ID_START_VALUE, which equals to -1.
     * If BodyIndex of the detector has a higher value, that is used for the body fetching. If the given body is restricted, no gesture recognition is done.
     * @param body
     * @param delta
     * @return Returns detected gesutre IDs sorted in ascending order
     */
    std::vector<UID> pollGestures(const BodyIndex &bodyIndex, const float &delta);

    /**
     * @brief This does the same as above function but breaks at the first sight of a gesture instead of polling it.
     * If the given body is restricted, no gesture recognition is done.
     * @param bodyIndex
     * @param delta
     * @return
     */
    UID determinePlayerPosture(const BodyIndex &bodyIndex, const float &delta);

    /**
     * @brief Since KinectHandler only directs visible bodies to us, return the size of m_Bodies
     * @return
     */
    size_t getDetectedBodyCount() const;

    /**
     * @brief Returns a bounding box whose origin is on the top left corner.
     * @param body
     * @return
     */
    BodyRect getBodyRect(IBody *body) const;

    /**
     * @brief Returns a bounding box whose origin is on the top left corner.
     * @param body
     * @return
     */
    BodyRect getHeadRect(IBody *body) const;

    float toDegree(const float &radian) const;
    float toRadian(const float &degree) const;

    Vector4 getJointOrientation(IBody *body, JointType joint) const;
    CameraSpacePoint getJointPosition(IBody *body, JointType joint) const;
    float getAngleBetweenTwoPoints(const CameraSpacePoint &pointOne, const CameraSpacePoint &pointTwo) const;

    /**
     * @brief Active hand is the one above the other
     * @param joints
     * @return
     */
    JointType getActiveHand(Joint *joints) const;

    /**
     * @brief Returns the closest body on Z-axis
     * @return
     */
    IBody *getClosestBody();

    /**
     * @brief Use this to disconnect any function from onBodyFound and onBodyLost
     */
    void resetBodyNotification();

    /**
     * @brief Retrieves a boolean value that indicates if the body is restricted from a full range of motion.
     * @param body
     * @return
     */
    bool isBodyRestricted(IBody *body) const;
    bool isBodyTracked(IBody *body) const;

    template<class Point3D>
    float getDistanceBetweenPoints(const Point3D &pointOne, const Point3D &pointTwo) const
    {
        return std::sqrtf(std::powf((pointTwo.X - pointOne.X), 2) + std::powf((pointTwo.Y - pointOne.Y), 2) + std::powf((pointTwo.Z - pointOne.Z), 2));
    }

    /**
     * @brief Adds a detector function, If the function is valid returns true otherwise returns false and doesn't add the function
     * @param detectFunc
     * @return
     */
    template<class DetectorClass>
    DetectorClass *addDetector(const BodyIndex &player, const UID &customID = 0)
    {
        DetectorClass *detector = new(std::nothrow) DetectorClass(*this, customID);
        if (detector) {
            detector->setBodyIndex(player);
            m_Detectors.push_back(detector);
            return detector;
        }
        return nullptr;
    }

    template<class DetectorClass>
    DetectorClass *getDetector(const UID &detectorID)
    {
        for (auto detector : m_Detectors) {
            if (detector->getUniqueID() == detectorID) {
                return dynamic_cast<DetectorClass *>(detector);
            }
        }
        return nullptr;
    }

private:
    KinectHandler *m_KinectHandler;
    UINT64 m_SensorTime;
    std::array<IBody *, BODY_COUNT> m_Bodies;
    std::vector<unsigned int> m_UIDs;

    /**
     * @brief m_DetectionFuncs - A detection function must take an IBody and a delta time. If a function succeeds, it returns the ID to the class it belongs.
     * Otherwise, it returns INVALID_UID
     */
    std::vector<DGestureBase *> m_Detectors;

private:
    void processBody(const std::array<IBody *, BODY_COUNT> &bodyArray, UINT64 sensorTime);
    void processBodyNotificitons(const std::array<IBody *, BODY_COUNT> &bodyArray);
    void resetBodies();
};

#endif // GEPOTOOL_H
