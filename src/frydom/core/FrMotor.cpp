//
// Created by Lucas Letournel on 24/07/18.
//

#include <chrono/physics/ChLinkMotorRotationSpeed.h>
#include "FrMotor.h"

#include "MathUtils/MathUtils.h"

namespace frydom{
    void make_rotoid_motorized(std::shared_ptr<FrBody> bodyMaster, std::shared_ptr<FrBody> bodySlave,
                               bool relpos, chrono::ChVector<> masterPos, chrono::ChVector<> slavePos,
                               chrono::ChVector<> masterVec, chrono::ChVector<> slaveVec) {
        try {
            if (bodyMaster->GetSystem()== nullptr)
                throw std::string("make_rotoid_motorized : bodyMaster does not belong to a system.");
            auto systemMaster = bodyMaster->GetSystem();
            if (bodySlave->GetSystem()== nullptr)
                throw std::string("make_rotoid_motorized : bodySlave does not belong to a system.");
            auto systemSlave = bodySlave->GetSystem();
            if (systemMaster!=systemSlave)
                throw std::string("make_rotoid_motorized : bodyMaster and bodySlave don't belong to the same system");
            // Create the motor
            auto rotmotor = std::make_shared<chrono::ChLinkMotorRotationSpeed>();
            rotmotor->Initialize(bodySlave, bodyMaster, relpos, slavePos, masterPos, slaveVec, masterVec);
            systemMaster->Add(rotmotor);

            // Create a ChFunction to be used for the ChLinkMotorRotationSpeed
            auto mwspeed = std::make_shared<chrono::ChFunction_Const>(
                    MU_PI_2); // constant angular speed, in [rad/s], 1PI/s =180�/s
            // Let the motor use this motion function:
            rotmotor->SetSpeedFunction(mwspeed);

        }
        catch(std::string const& error_msg)
        {
            fmt::print(error_msg);
        }

    }
}