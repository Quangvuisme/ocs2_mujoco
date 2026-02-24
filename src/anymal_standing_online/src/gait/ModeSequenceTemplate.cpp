#include "anymal_standing_online/gait/ModeSequenceTemplate.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LoadData.h>


namespace anymal_standing_online {
    std::ostream &operator<<(std::ostream &stream, const ModeSequenceTemplate &modeSequenceTemplate) {
        stream << "Template switching times: {" << ocs2::toDelimitedString(modeSequenceTemplate.switchingTimes) << "}\n";
        stream << "Template mode sequence:   {" << ocs2::toDelimitedString(modeSequenceTemplate.modeSequence) << "}\n";
        return stream;
    }


    ModeSequenceTemplate loadModeSequenceTemplate(const std::string &filename, const std::string &topicName,
                                                  bool verbose) {
        std::vector<ocs2::scalar_t> switchingTimes;
        ocs2::loadData::loadStdVector(filename, topicName + ".switchingTimes", switchingTimes, verbose);

        std::vector<std::string> modeSequenceString;
        ocs2::loadData::loadStdVector(filename, topicName + ".modeSequence", modeSequenceString, verbose);

        if (switchingTimes.empty() || modeSequenceString.empty()) {
            throw std::runtime_error("[loadModeSequenceTemplate] failed to load : " + topicName + " from " + filename);
        }

        // convert the mode name to mode enum
        std::vector<size_t> modeSequence;
        modeSequence.reserve(modeSequenceString.size());
        for (const auto &modeName: modeSequenceString) {
            modeSequence.push_back(string2ModeNumber(modeName));
        }

        return {switchingTimes, modeSequence};
    }


    Gait toGait(const ModeSequenceTemplate &modeSequenceTemplate) {
        const auto startTime = modeSequenceTemplate.switchingTimes.front();
        const auto endTime = modeSequenceTemplate.switchingTimes.back();
        Gait gait;
        gait.duration = endTime - startTime;
        // Events: from time -> phase
        gait.eventPhases.reserve(modeSequenceTemplate.switchingTimes.size());
        std::for_each(modeSequenceTemplate.switchingTimes.begin() + 1, modeSequenceTemplate.switchingTimes.end() - 1,
                      [&](ocs2::scalar_t eventTime) { gait.eventPhases.push_back((eventTime - startTime) / gait.duration); });
        // Modes:
        gait.modeSequence = modeSequenceTemplate.modeSequence;
        assert(isValidGait(gait));
        return gait;
    }


    ocs2::ModeSchedule loadModeSchedule(const std::string &filename, const std::string &topicName, bool verbose) {
        std::vector<ocs2::scalar_t> eventTimes;
        ocs2::loadData::loadStdVector(filename, topicName + ".eventTimes", eventTimes, verbose);

        std::vector<std::string> modeSequenceString;
        ocs2::loadData::loadStdVector(filename, topicName + ".modeSequence", modeSequenceString, verbose);

        if (modeSequenceString.empty()) {
            throw std::runtime_error("[loadModeSchedule] failed to load : " + topicName + " from " + filename);
        }

        // convert the mode name to mode enum
        std::vector<size_t> modeSequence;
        modeSequence.reserve(modeSequenceString.size());
        for (const auto &modeName: modeSequenceString) {
            modeSequence.push_back(string2ModeNumber(modeName));
        }

        return {eventTimes, modeSequence};
    }
} // namespace anymal_standing_online
