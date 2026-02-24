#pragma once

#include <ocs2_core/reference/ModeSchedule.h>

#include "quadruped_nmpc/gait/ModeSequenceTemplate.h"


namespace quadruped_nmpc {
    class GaitSchedule {
    public:
        GaitSchedule(ocs2::ModeSchedule initModeSchedule, ModeSequenceTemplate initModeSequenceTemplate,
                     ocs2::scalar_t phaseTransitionStanceTime);

        /**
         * Sets the mode schedule.
         *
         * @param [in] modeSchedule: The mode schedule to be used.
         */
        void setModeSchedule(const ocs2::ModeSchedule &modeSchedule) { modeSchedule_ = modeSchedule; }

        /**
         * Gets the mode schedule.
         *
         * @param [in] lowerBoundTime: The smallest time for which the ModeSchedule should be defined.
         * @param [in] upperBoundTime: The greatest time for which the ModeSchedule should be defined.
         */
        ocs2::ModeSchedule getModeSchedule(ocs2::scalar_t lowerBoundTime, ocs2::scalar_t upperBoundTime);

        /**
         * Used to insert a new user defined logic in the given time period.
         *
         * @param [in] startTime: The initial time from which the new mode sequence template should start.
         * @param [in] finalTime: The final time until when the new mode sequence needs to be defined.
         */
        void insertModeSequenceTemplate(const ModeSequenceTemplate &modeSequenceTemplate, ocs2::scalar_t startTime,
                                        ocs2::scalar_t finalTime);

    private:
        /**
         * Extends the switch information from lowerBoundTime to upperBoundTime based on the template mode sequence.
         *
         * @param [in] startTime: The initial time from which the mode schedule should be appended with the template.
         * @param [in] finalTime: The final time to which the mode schedule should be appended with the template.
         */
        void tileModeSequenceTemplate(ocs2::scalar_t startTime, ocs2::scalar_t finalTime);

        ocs2::ModeSchedule modeSchedule_;
        ModeSequenceTemplate modeSequenceTemplate_;
        ocs2::scalar_t phaseTransitionStanceTime_;
    };
} // namespace quadruped_nmpc
