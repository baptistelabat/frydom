//
// Created by frongere on 23/01/19.
//

#include "FrAngularActuatorVelocity.h"


#include "frydom/core/body/FrBody.h"
#include "frydom/core/link/FrLinkBase.h"

#include "frydom/core/math/functions/FrFunctionsInc.h"


namespace frydom {

    namespace internal {

        FrLinkMotorRotationSpeed::FrLinkMotorRotationSpeed(FrAngularActuatorVelocity *frydomActuator) :
            m_frydomActuator(frydomActuator) {
            // No constraints are added by this motor as it is attached to an other link that realizes the link
            SetSpindleConstraint(chrono::ChLinkMotorRotation::SpindleConstraint::FREE);

        }

        void FrLinkMotorRotationSpeed::SetupInitial() {
            // Based on ChLinkMateGeneric::Initialize

            this->Body1 = m_frydomActuator->GetChronoBody1().get();
            this->Body2 = m_frydomActuator->GetChronoBody2().get();

            this->mask->SetTwoBodiesVariables(&Body1->Variables(), &Body2->Variables());

            this->frame1 = internal::FrFrame2ChFrame(m_frydomActuator->GetNode1()->GetFrameWRT_COG_InBody());
            this->frame2 = internal::FrFrame2ChFrame(m_frydomActuator->GetNode2()->GetFrameWRT_COG_InBody());
        }

//        bool FrLinkMotorRotationSpeed::GetDisabled() {
//            return IsDisabled();
//        }
//
//        void FrLinkMotorRotationSpeed::MakeDisabled(bool disabled) {
//            SetDisabled(disabled);
//        }


    }  // end namespace frydom::internal




    FrAngularActuatorVelocity::FrAngularActuatorVelocity(FrLink *actuatedLink) : FrAngularActuator(actuatedLink) {
        m_chronoActuator = std::make_shared<internal::FrLinkMotorRotationSpeed>(this);
    }

    void FrAngularActuatorVelocity::SetConstantAngularVelocity(double velocity) {
        GetChronoItem_ptr()->SetSpeedFunction(std::make_shared<chrono::ChFunction_Const>(velocity));
    }

    void FrAngularActuatorVelocity::SetAngularVelocityFunction(const FrFunctionBase& function) {

        // TODO : ici, on definit la fonction de pilotage de la vitesse
        auto chronoFunctionInterface = internal::FrFunctionChronoInterface(function);

        GetChronoItem_ptr()->SetSpeedFunction(chronoFunctionInterface.GetChronoFunction());


    }

    void FrAngularActuatorVelocity::Initialize() {
        m_chronoActuator->SetupInitial();
    }

    void FrAngularActuatorVelocity::Update(double time) {

    }


//    internal::FrLinkMotorRotationSpeed* FrAngularActuatorVelocity::GetChronoActuator() const {
//        return m_chronoActuator.get();
//    }

    std::shared_ptr<chrono::ChLink> FrAngularActuatorVelocity::GetChronoLink() {
        return std::dynamic_pointer_cast<chrono::ChLink>(m_chronoActuator);
    }

    internal::FrLinkMotorRotationSpeed *FrAngularActuatorVelocity::GetChronoItem_ptr() const {
        return m_chronoActuator.get();
    }

    void FrAngularActuatorVelocity::SetMotorFunction(const FrFunctionBase &function) {
        SetAngularVelocityFunction(function);
    }


}  // end namespace frydom
