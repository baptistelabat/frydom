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

//            bool GetDisabled() override;
//
//            void MakeDisabled(bool disabled) override;


        };

    }  // end namespace frydom::internal


    // Forward declaration
    class FrFunctionBase;

    class FrAngularActuatorVelocity : public FrAngularActuator {

    private:
        std::shared_ptr<internal::FrLinkMotorRotationSpeed> m_chronoActuator;


    public:
        explicit FrAngularActuatorVelocity(FrLink* actuatedLink);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "AngularActuatorVelocity"; }

        void SetConstantAngularVelocity(double velocity);

        void SetAngularVelocityFunction(const FrFunctionBase& function);

        void Initialize() override;

        void Update(double time);

        void StepFinalize() override;


        friend void internal::FrLinkMotorRotationSpeed::SetupInitial();


    protected:

        void SetMotorFunction(const FrFunctionBase& function) override;

//        internal::FrLinkMotorRotationSpeed* GetChronoActuator() const override;
        std::shared_ptr<chrono::ChLink> GetChronoLink() override;
        internal::FrLinkMotorRotationSpeed* GetChronoItem_ptr() const override;

    };





}  // end namespace frydom

#endif //FRYDOM_FRANGULARACTUATORVELOCITY_H
