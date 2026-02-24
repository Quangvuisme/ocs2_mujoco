#pragma once

#include <ocs2_core/reference/ModeSchedule.h>

#include "quadruped_nmpc/common/Types.h"

namespace quadruped_nmpc {
    struct LegPhase {
        ocs2::scalar_t phase;
        ocs2::scalar_t duration;
    };

    struct ContactTiming {
        ocs2::scalar_t start;
        ocs2::scalar_t end;
    };

    struct SwingTiming {
        ocs2::scalar_t start;
        ocs2::scalar_t end;
    };

    /**
    * @brief Get the contact phase for all legs.
    * If leg in contact, returns a value between 0.0 (at start of contact phase) and 1.0 (at end of contact phase).
    * If leg not in contact (i.e. in swing), returns -1.0.
    * If mode schedule starts with contact phase, returns 1.0 during this phase.
    * If mode schedule ends with contact phase, returns 0.0 during this phase.
    * @param [in] time : Query time.
    * @param [in] modeSchedule : Mode schedule.
    * @return Contact phases for all legs.
    */
    feet_array_t<LegPhase> getContactPhasePerLeg(ocs2::scalar_t time, const ocs2::ModeSchedule &modeSchedule);

    /**
    * @brief Get the swing phase for all legs.
    * If leg in swing, returns a value between 0.0 (at start of swing phase) and 1.0 (at end of swing phase).
    * If leg not in swing (i.e. in contact), returns -1.0.
    * If mode schedule starts with swing phase, returns 1.0 during this phase.
    * If mode schedule ends with swing phase, returns 0.0 during this phase.
    * @param [in] time : Query time.
    * @param [in] modeSchedule : Mode schedule.
    * @return Swing phases for all legs.
    */
    feet_array_t<LegPhase> getSwingPhasePerLeg(ocs2::scalar_t time, const ocs2::ModeSchedule &modeSchedule);

    /** Extracts the contact timings for all legs from a modeSchedule */
    feet_array_t<std::vector<ContactTiming> > extractContactTimingsPerLeg(const ocs2::ModeSchedule &modeSchedule);

    /** Extracts the swing timings for all legs from a modeSchedule */
    feet_array_t<std::vector<SwingTiming> > extractSwingTimingsPerLeg(const ocs2::ModeSchedule &modeSchedule);

    /** Returns time of the next lift off. Returns nan if leg is not lifting off */
    ocs2::scalar_t getTimeOfNextLiftOff(ocs2::scalar_t currentTime, const std::vector<ContactTiming> &contactTimings);

    /** Returns time of the  touch down for all legs from a modeschedule. Returns nan if leg does not touch down */
    ocs2::scalar_t getTimeOfNextTouchDown(ocs2::scalar_t currentTime, const std::vector<ContactTiming> &contactTimings);

    /**
    * Get {startTime, endTime} for all contact phases. Swingphases are always implied in between: endTime[i] < startTime[i+1]
    * times are NaN if they cannot be identified at the boundaries
    * Vector is empty if there are no contact phases
    */
    std::vector<ContactTiming> extractContactTimings(const std::vector<ocs2::scalar_t> &eventTimes,
                                                     const std::vector<bool> &contactFlags);

    /**
    * Get {startTime, endTime} for all swing phases. Contact phases are always implied in between: endTime[i] < startTime[i+1]
    * times are NaN if they cannot be identified at the boundaries
    * Vector is empty if there are no swing phases
    */
    std::vector<SwingTiming> extractSwingTimings(const std::vector<ocs2::scalar_t> &eventTimes,
                                                 const std::vector<bool> &contactFlags);

    /**
    * Extracts for each leg the contact sequence over the motion phase sequence.
    * @param modeSequence : Sequence of contact modes.
    * @return Sequence of contact flags per leg.
    */
    feet_array_t<std::vector<bool> > extractContactFlags(const std::vector<size_t> &modeSequence);
}
