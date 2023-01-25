#include "HandModel.h"
#include "Utils/Utils.h"

static constexpr float GRAVITY = 9.807f;

static Quaternion orientationFromGravity(const Eigen::Vector3d &gravity);
static Eigen::Vector3d anglesFromGravity(const Eigen::Vector3d &gravity);

HandModel::HandModel()
{
    for (uint8_t i=0; i < 16; i++)
        joints[i] = Quaternion();
    
    for (uint8_t i=0; i < 16; i++)
        joints_offset[i] = Quaternion();
    
    // Wrist JSON
    JsonObject wristObj = encoded.createNestedObject("wrist");
    hand.wrist.serialize(wristObj);
    // Fingers JSON
    JsonArray fingersArr = encoded.createNestedArray("fingers");
    for(uint8_t i=0; i < 5; i++)
    {
        JsonObject fingerObj = fingersArr.createNestedObject();
        JsonArray jointsArr = fingerObj.createNestedArray("joints");
        for(uint8_t j=0; j < 3; j++)
        {
            hand.fingers[i].joints[j].serialize(jointsArr);
        }
    }
}

HandModel::FingerPose& HandModel::getFinger(uint8_t index)
{
    return hand.fingers[index];
}

Quaternion& HandModel::getWrist()
{
    return hand.wrist;
}

Quaternion& HandModel::getJoint(uint8_t index)
{
    return joints[index];
}

HandModel::FingerPose& HandModel::getOffsetFinger(uint8_t index)
{
    return hand_offset.fingers[index];
}

Quaternion& HandModel::getOffsetWrist()
{
    return hand_offset.wrist;
}

Quaternion& HandModel::getOffset(uint8_t index)
{
    return joints_offset[index];
}


void HandModel::serialize(String &outStr)
{
    union HandOrientation
    {
        HandModel::HandPose hand;
        Quaternion joints[16];

        HandOrientation() {}
    } hand_corr;

    for(uint8_t i=0; i < 16; i++)
    {
        hand_corr.joints[i] = joints_offset[i] * joints[i];
    }

    encoded["wrist"]["x"] = hand_corr.hand.wrist.x();
    encoded["wrist"]["y"] = hand_corr.hand.wrist.y();
    encoded["wrist"]["z"] = hand_corr.hand.wrist.z();
    encoded["wrist"]["w"] = hand_corr.hand.wrist.w();

    JsonArray fingersArr = encoded["fingers"];
    Quaternion corr_wrist_inv = hand_corr.hand.wrist.inverse();
    for(uint8_t i=0; i < 5; i++)
    {
        JsonArray jointsArr = fingersArr[i]["joints"];
        Quaternion wrist_diff = corr_wrist_inv * hand_corr.hand.fingers[i].joints[0];
        jointsArr[0]["x"] = wrist_diff.x();
        jointsArr[0]["y"] = wrist_diff.y();
        jointsArr[0]["z"] = wrist_diff.z();
        jointsArr[0]["w"] = wrist_diff.w();

        for(uint8_t j=1; j < 3; j++)
        {
            // Relative rotation between finger joints
            Quaternion diff = hand_corr.hand.fingers[i].joints[j-1].inverse() * hand_corr.hand.fingers[i].joints[j];
            jointsArr[j]["x"] = diff.x();
            jointsArr[j]["y"] = diff.y();
            jointsArr[j]["z"] = diff.z();
            jointsArr[j]["w"] = diff.w();
        }
    }

    serializeJson(encoded, outStr);
}

inline Quaternion orientationFromGravity(const Eigen::Vector3d &gravity)
{
    return Quaternion(anglesFromGravity(gravity));
}

inline Eigen::Vector3d anglesFromGravity(const Eigen::Vector3d &gravity)
{
    // https://youtu.be/CHSYgLfhwUo?t=1427
    const double ax = gravity.x();
    const double ay = gravity.y();
    const double az = gravity.z();
    const double ex = atan2(ay, az);
    const double ey = atan2(-1 * ax, sqrt(ay*ay + az*az));
    const double ez = 0;

    return Eigen::Vector3d(ex, ey, ez);
}

void HandModel::initializeJoint(uint8_t index, const Eigen::Vector3d &gravity)
{
    initializeJoint(joints[index], gravity);
}

void HandModel::initializeJoint(Quaternion &joint, const Eigen::Vector3d &gravity)
{
    joint = orientationFromGravity(gravity);
}

void HandModel::offsetJoint(uint8_t index, const Eigen::Vector3d &gravity)
{
    joints_offset[index] = orientationFromGravity(gravity).inverse();
}

void HandModel::offsetJoint(uint8_t index, const Quaternion &orientation)
{
    joints_offset[index] = orientation.inverse();
}

void HandModel::updateJoint(Quaternion &joint, const Eigen::Vector3d &dEuler, const Eigen::Vector3d &gravity)
{
    // https://ahrs.readthedocs.io/en/latest/filters.html
    // https://www.mdpi.com/1424-8220/15/8/19302/htm
    const float em = abs_tp(gravity.norm() - GRAVITY) / GRAVITY;
    float gain_factor;
    if(em <= ERROR_T1)
        gain_factor = 1;
    else if(em >= ERROR_T2)
        gain_factor = 0;
    else {
        gain_factor = (ERROR_T2 - em) / ERROR_T1;
    }
    const float ADAPTIVE_GAIN = STATIC_GAIN * gain_factor;

    const Quaternion &q = joint;
    const Eigen::Vector3d &e = dEuler;
    // Attitude propagation
    Quaternion predicted_w;
    predicted_w.w() = q.w() - e.x()/2 * q.x() - e.y()/2 * q.y() - e.z()/2 * q.z();
    predicted_w.x() = q.x() + e.x()/2 * q.w() - e.y()/2 * q.z() + e.z()/2 * q.y();
    predicted_w.y() = q.y() + e.x()/2 * q.z() + e.y()/2 * q.w() - e.z()/2 * q.x();
    predicted_w.z() = q.z() - e.x()/2 * q.y() + e.y()/2 * q.x() + e.z()/2 * q.w();
    Eigen::Vector3d predicted_g = predicted_w * gravity;
    // Attitude correction
    Quaternion dqacc = Quaternion::FromTwoVectors(predicted_g, Eigen::Vector3d(0, 0, 1));
    dqacc = Quaternion() * (1-ADAPTIVE_GAIN) + dqacc * ADAPTIVE_GAIN;
    dqacc.normalize();

    joint = predicted_w * dqacc;
}

void HandModel::updateJoint(uint8_t index, const Eigen::Vector3d &dEuler, const Eigen::Vector3d &gravity)
{
    updateJoint(joints[index], dEuler, gravity);
}

void HandModel::updateFinger(FingerId id, const Eigen::Vector3d dEuler[], const Eigen::Vector3d gravity[])
{
    FingerPose &finger = hand.fingers[id];
    for(uint8_t i = 0; i < 3; i++)
    {
        updateJoint(finger.joints[i], dEuler[i], gravity[i]);
    } 
}

void HandModel::updateWrist(const Eigen::Vector3d &dEuler, const Eigen::Vector3d &gravity)
{
    updateJoint(hand.wrist, dEuler, gravity);
}
