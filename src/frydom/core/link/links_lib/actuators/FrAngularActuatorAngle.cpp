// ==========================================================================
// FRyDoM - frydom-ce.org
// 
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
// 
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
// 
// ==========================================================================


#include "FrAngularActuatorAngle.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/core/link/FrLinkBase.h"

#include "frydom/core/math/functions/FrFunctionsInc.h"


namespace frydom {
    
    namespace internal {


        FrLinkMotorRotationAngle::FrLinkMotorRotationAngle(FrAngularActuatorAngle *frydomActuator):
                m_frydomActuator(frydomActuator) {
            // No constraints are added by this motor as it is attached to an other link that realizes the link
            SetSpindleConstraint(chrono::ChLinkMotorRotation::SpindleConstraint::FREE);
        }
        
        void FrLinkMotorRotationAngle::SetupInitial() {
            // Based on ChLinkMateGeneric::Initialize

            this->Body1 = m_frydomActuator->GetChronoBody1().get();
            this->Body2 = m_frydomActuator->GetChronoBody2().get();

            this->mask->SetTwoBodiesVariables(&Body1->Variables(), &Body2->Variables());

            this->frame1 = internal::FrFrame2ChFrame(m_frydomActuator->GetNode1()->GetFrameWRT_COG_InBody());
            this->frame2 = internal::FrFrame2ChFrame(m_frydomActuator->GetNode2()->GetFrameWRT_COG_InBody());
        }

//        bool FrLinkMotorRotationAngle::GetDisabled() {
//            return IsDisabled();
//        }
//
//        void FrLinkMotorRotationAngle::MakeDisabled(bool disabled) {
//            SetDisabled(disabled);
//        }
        
    }


    FrAngularActuatorAngle::FrAngularActuatorAngle(FrLink *actuatedLink) : FrAngularActuator(actuatedLink) {
        m_chronoActuator = std::make_shared<internal::FrLinkMotorRotationAngle>(this);
    }

    void FrAngularActuatorAngle::SetConstantAngularAngle(double velocity) {
        GetChronoItem_ptr()->SetMotorFunction(std::make_shared<chrono::ChFunction_Const>(velocity));
    }

    void FrAngularActuatorAngle::SetAngularAngleFunction(const FrFunctionBase &function) {

        // TODO : ici, on definit la fonction de pilotage de la vitesse
        auto chronoFunctionInterface = internal::FrFunctionChronoInterface(function);

        GetChronoItem_ptr()->SetAngleFunction(chronoFunctionInterface.GetChronoFunction());


    }

    void FrAngularActuatorAngle::Initialize() {
        m_chronoActuator->SetupInitial();
    }

    void FrAngularActuatorAngle::Update(double time) {

    }

    void FrAngularActuatorAngle::StepFinalize() {

    }


//    internal::FrLinkMotorRotationAngle* FrAngularActuatorAngle::GetChronoActuator() const {
//        return m_chronoActuator.get();
//    }

    std::shared_ptr<chrono::ChLink> FrAngularActuatorAngle::GetChronoLink() {
        return std::dynamic_pointer_cast<chrono::ChLink>(m_chronoActuator);
    }

    internal::FrLinkMotorRotationAngle *FrAngularActuatorAngle::GetChronoItem_ptr() const {
        return m_chronoActuator.get();
    }

    void FrAngularActuatorAngle::SetMotorFunction(const FrFunctionBase &function) {
        SetAngularAngleFunction(function);
    }
    
    
    
}  // end namespace frydom
