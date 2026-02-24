#include <iostream>
#include <memory>
#include <string>

#include <anymal_walking_online/LeggedRobotInterface.h>
#include <anymal_walking_online/package_path.h>
#include <pinocchio/multibody/model.hpp>

using namespace anymal_walking_online;

int main(int argc, char** argv) {
    try {
        const bool verbose = true;
        const std::string taskFile = getPath() + "/config/mpc/task.info";
        const std::string urdfFile = getPath() + "/robots/anymal_c/urdf/anymal.urdf";
        const std::string referenceFile = getPath() + "/config/command/reference.info";

        std::cout << "===============================" << std::endl;
        std::cout << "  ANYmal C Interface Test      " << std::endl;
        std::cout << "===============================" << std::endl;
        std::cout << "Task file: " << taskFile << std::endl;
        std::cout << "URDF file: " << urdfFile << std::endl;
        std::cout << "Reference file: " << referenceFile << std::endl;

        bool useHardFrictionConeConstraint = true;

        LeggedRobotInterface anymalInterface(taskFile, urdfFile, referenceFile, useHardFrictionConeConstraint);

        const auto& info = anymalInterface.getCentroidalModelInfo();
        std::cout << "\nCentroidal Model Info:" << std::endl;
        std::cout << " - stateDim: " << info.stateDim << std::endl;
        std::cout << " - inputDim: " << info.inputDim << std::endl;
        std::cout << " - actuatedDofNum: " << info.actuatedDofNum << std::endl;
        std::cout << " - numThreeDofContacts: " << info.numThreeDofContacts << std::endl;
        std::cout << " - numSixDofContacts: " << info.numSixDofContacts << std::endl;
        std::cout << " - robotMass: " << info.robotMass << std::endl;

        // Print Pinocchio joint order
        const auto& model = anymalInterface.getPinocchioInterface().getModel();
        std::cout << "\n=== PINOCCHIO JOINT ORDER ===" << std::endl;
        std::cout << "nq (config dim): " << model.nq << std::endl;
        std::cout << "nv (velocity dim): " << model.nv << std::endl;
        std::cout << "\nJoint names in Pinocchio model:" << std::endl;
        for (size_t i = 0; i < model.names.size(); ++i) {
            std::cout << "  [" << i << "] " << model.names[i];
            if (i < model.njoints) {
                std::cout << " (idx_q=" << model.idx_qs[i] << ", nq=" << model.nqs[i] << ")";
            }
            std::cout << std::endl;
        }

        // Print end effector frame indices
        std::cout << "\n=== END EFFECTOR FRAMES ===" << std::endl;
        std::cout << "Contact frames indices: ";
        for (auto idx : info.endEffectorFrameIndices) {
            std::cout << idx << " ";
        }
        std::cout << std::endl;
        std::cout << "Contact frame names:" << std::endl;
        for (auto idx : info.endEffectorFrameIndices) {
            std::cout << "  [" << idx << "] " << model.frames[idx].name << std::endl;
        }

        std::cout << "\n=== INITIAL STATE ===" << std::endl;
        const auto& initState = anymalInterface.getInitialState();
        std::cout << "Full state (" << initState.size() << "):" << std::endl;
        std::cout << initState.transpose() << std::endl;
        
        std::cout << "\nJoint positions (indices 12-23):" << std::endl;
        for (int i = 12; i < 24; ++i) {
            std::cout << "  [" << i << "] " << initState(i) << std::endl;
        }

        std::cout << "\nInitialization complete!" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
