//
// Created by frongere on 23/01/19.
//

#ifndef FRYDOM_FRANGULARACTUATORVELOCITY_H
#define FRYDOM_FRANGULARACTUATORVELOCITY_H


#include "FrAngularActuator.h"

#include "chrono/physics/ChLinkMotorRotationSpeed.h"


namespace frydom {


    // Forward declaration
    class FrAngularActuatorVelocity;

    namespace internal {

        struct FrLinkMotorRotationSpeed : public chrono::ChLinkMotorRotationSpeed {

            FrAngularActuatorVelocity* m_frydomActuator;

            explicit FrLinkMotorRotationSpeed(FrAngularActuatorVelocity* frydomActuator);

            void SetupInitial() override;


        };

    }  // end namespace frydom::internal


    class FrAngularActuatorVelocity : public FrAngularActuator {

    private:
        std::shared_ptr<internal::FrLinkMotorRotationSpeed> m_chronoActuator;


    public:
        explicit FrAngularActuatorVelocity(FrLink_* associatedLink);











        friend void internal::FrLinkMotorRotationSpeed::SetupInitial();

    };


//    std::shared_ptr<FrAngularActuatorVelocity> make_angular_actuator_velocity(Fr)



}  // end namespace frydom

#endif //FRYDOM_FRANGULARACTUATORVELOCITY_H
