#pragma once

#include <ocs2_core/reference/ModeSchedule.h>

#include "anymal_standing_online/common/Types.h"
#include "anymal_standing_online/foot_planner/SplineCpg.h"

namespace anymal_standing_online {
    class SwingTrajectoryPlanner {
    public:
        struct Config {
            ocs2::scalar_t liftOffVelocity = 0.0;
            ocs2::scalar_t touchDownVelocity = 0.0;
            ocs2::scalar_t swingHeight = 0.1;
            ocs2::scalar_t swingTimeScale = 0.15;
            // swing phases shorter than this time will be scaled down in height and velocity
        };

        SwingTrajectoryPlanner(Config config, size_t numFeet);

        void update(const ocs2::ModeSchedule &modeSchedule, ocs2::scalar_t terrainHeight);

        void update(const ocs2::ModeSchedule &modeSchedule, const feet_array_t<ocs2::scalar_array_t> &liftOffHeightSequence,
                    const feet_array_t<ocs2::scalar_array_t> &touchDownHeightSequence);

        ocs2::scalar_t getZvelocityConstraint(size_t leg, ocs2::scalar_t time) const;

        ocs2::scalar_t getZpositionConstraint(size_t leg, ocs2::scalar_t time) const;

    private:
        /**
         * Extracts for each leg the contact sequence over the motion phase sequence.
         * @param phaseIDsStock
         * @return contactFlagStock
         */
        feet_array_t<std::vector<bool> > extractContactFlags(const std::vector<size_t> &phaseIDsStock) const;

        /**
         * Finds the take-off and touch-down times indices for a specific leg.
         *
         * @param index
         * @param contactFlagStock
         * @return {The take-off time index for swing legs, touch-down time index for swing legs}
         */
        static std::pair<int, int> findIndex(size_t index, const std::vector<bool> &contactFlagStock);

        /**
         * based on the input phaseIDsStock finds the start subsystem and final subsystem of the swing
         * phases of the a foot in each subsystem.
         *
         * startTimeIndexStock: eventTimes[startTimesIndex] will be the take-off time for the requested leg.
         * finalTimeIndexStock: eventTimes[finalTimesIndex] will be the touch-down time for the requested leg.
         *
         * @param [in] footIndex: Foot index
         * @param [in] phaseIDsStock: The sequence of the motion phase IDs.
         * @param [in] contactFlagStock: The sequence of the contact status for the requested leg.
         * @return { startTimeIndexStock, finalTimeIndexStock}
         */
        static std::pair<std::vector<int>, std::vector<int> > updateFootSchedule(
            const std::vector<bool> &contactFlagStock);

        /**
         * Check if event time indices are valid
         * @param leg
         * @param index : phase index
         * @param startIndex : liftoff event time index
         * @param finalIndex : touchdown event time index
         * @param phaseIDsStock : mode sequence
         */
        static void checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                             const std::vector<size_t> &phaseIDsStock);

        static ocs2::scalar_t swingTrajectoryScaling(ocs2::scalar_t startTime, ocs2::scalar_t finalTime, ocs2::scalar_t swingTimeScale);

        const Config config_;
        const size_t numFeet_;

        feet_array_t<std::vector<SplineCpg> > feetHeightTrajectories_;
        feet_array_t<std::vector<ocs2::scalar_t> > feetHeightTrajectoriesEvents_;
    };

    SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string &fileName,
                                                               const std::string &fieldName = "swing_trajectory_config",
                                                               bool verbose = true);
}
