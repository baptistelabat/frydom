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

        struct FrLinkMotorRotationSpeed : public FrMotorBase, public chrono::ChLinkMotorRotationSpeed {

            FrAngularActuatorVelocity* m_frydomActuator;

            explicit FrLinkMotorRotationSpeed(FrAngularActuatorVelocity* frydomActuator);

            void SetupInitial() override;

            bool GetDisabled() override;

            void MakeDisabled(bool disabled) override;


        };

    }  // end namespace frydom::internal


    // Forward declaration
    class FrFunctionBase;

    class FrAngularActuatorVelocity : public FrAngularActuator {

    private:
        std::shared_ptr<internal::FrLinkMotorRotationSpeed> m_chronoActuator;

//        std::shared_ptr<FrFunctionInterface> m_function;


    public:
        explicit FrAngularActuatorVelocity(FrLink* actuatedLink);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "AngularActuatorVelocity"; }

        void SetConstantAngularVelocity(double velocity);

        void SetAngularVelocityFunction(std::shared_ptr<FrFunctionBase> function);

        void Initialize() override;

        void Update(double time);

        void StepFinalize() override;






        friend void internal::FrLinkMotorRotationSpeed::SetupInitial();


    protected:
        internal::FrLinkMotorRotationSpeed* GetChronoElement();
        std::shared_ptr<chrono::ChLink> GetChronoLink() override;

    };





}  // end namespace frydom

#endif //FRYDOM_FRANGULARACTUATORVELOCITY_H
