//
// Created by frongere on 23/01/19.
//

#include "FrAngularActuatorVelocity.h"


#include "frydom/core/body/FrBody.h"
#include "frydom/core/link/FrLinkBase.h"


namespace frydom {

    namespace internal {

        FrLinkMotorRotationSpeed::FrLinkMotorRotationSpeed(FrAngularActuatorVelocity *frydomActuator) :
            m_frydomActuator(frydomActuator) {

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


    }  // end namespace frydom::internal



    FrAngularActuatorVelocity::FrAngularActuatorVelocity(FrLink_ *associatedLink) : FrAngularActuator(associatedLink) {

    }



}  // end namespace frydom
