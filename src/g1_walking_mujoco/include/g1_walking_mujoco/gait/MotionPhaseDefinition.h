#pragma once

#include <map>
#include <string>
#include <boost/property_tree/ptree.hpp>

#include "g1_walking_mujoco/common/Types.h"

namespace g1_walking {

/**
 * G1 biped contact modes. Every foot has two 3-DoF contact points (toe and heel), in the order of
 * ModelSettings::contactNames3DoF: {l_foot_toe, r_foot_toe, l_foot_heel, r_foot_heel}.
 */
enum ModeNumber {
    FLY = 0,       // no foot on the ground
    LCONTACT = 1,  // left foot on the ground (right foot swings)
    RCONTACT = 2,  // right foot on the ground (left foot swings)
    STANCE = 3,    // double support
};

inline contact_flag_t modeNumber2StanceLeg(const size_t &modeNumber) {
    switch (modeNumber) {
        case LCONTACT: return contact_flag_t{true, false, true, false};
        case RCONTACT: return contact_flag_t{false, true, false, true};
        case STANCE:   return contact_flag_t{true, true, true, true};
        case FLY:
        default:       return contact_flag_t{false, false, false, false};
    }
}

inline size_t stanceLeg2ModeNumber(const contact_flag_t &stanceLegs) {
    const bool left = stanceLegs[0] || stanceLegs[2];
    const bool right = stanceLegs[1] || stanceLegs[3];
    return static_cast<size_t>(left) + 2 * static_cast<size_t>(right);
}

inline std::string modeNumber2String(const size_t &modeNumber) {
    std::map<size_t, std::string> modeToName;
    modeToName[FLY] = "FLY";
    modeToName[LCONTACT] = "LCONTACT";
    modeToName[RCONTACT] = "RCONTACT";
    modeToName[STANCE] = "STANCE";
    return modeToName[modeNumber];
}

inline size_t string2ModeNumber(const std::string &modeString) {
    std::map<std::string, size_t> nameToMode;
    nameToMode["FLY"] = FLY;
    nameToMode["LCONTACT"] = LCONTACT;
    nameToMode["RCONTACT"] = RCONTACT;
    nameToMode["STANCE"] = STANCE;
    return nameToMode[modeString];
}

}  // namespace g1_walking
