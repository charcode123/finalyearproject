#pragma once
#include "Quaternion.hpp"
#include "ArduinoJson.hpp"
#include <stdint.h>

class HandModel
{
public:
    enum FingerId {
        THUMB = 0, INDEX, MIDDLE, RING, PINKY
    };

    struct FingerPose {
        Quaternion joints[3];
    };

    struct HandPose
    {
        Quaternion wrist;
        FingerPose fingers[5];
    };

    static constexpr float STATIC_GAIN = 1-.995f;
    static constexpr float ERROR_T1 = .1f;
    static constexpr float ERROR_T2 = .2f;

public:
    HandModel();

    FingerPose& getFinger(uint8_t index);
    Quaternion& getWrist();
    Quaternion& getJoint(uint8_t index);

    FingerPose& getOffsetFinger(uint8_t index);
    Quaternion& getOffsetWrist();
    Quaternion& getOffset(uint8_t index);

    void serialize(String &outStr);

    void initializeJoint(uint8_t index, const Eigen::Vector3d &gravity);
    void initializeJoint(Quaternion &joint, const Eigen::Vector3d &gravity);

    void offsetJoint(uint8_t index, const Eigen::Vector3d &gravity);
    void offsetJoint(uint8_t index, const Quaternion &orientation);

    void updateFinger(FingerId id, const Eigen::Vector3d dEuler[], const Eigen::Vector3d gravity[]);
    void updateWrist(const Eigen::Vector3d &dEuler, const Eigen::Vector3d &gravity);
    
    void updateJoint(uint8_t index, const Eigen::Vector3d &dEuler, const Eigen::Vector3d &gravity);
    void updateJoint(Quaternion &joint, const Eigen::Vector3d &dEuler, const Eigen::Vector3d &gravity);

private:

    union
    {
        HandPose hand;
        Quaternion joints[16];
    };

    union
    {
        HandPose hand_offset;
        Quaternion joints_offset[16];
    };
    StaticJsonDocument<2048> encoded;
};
