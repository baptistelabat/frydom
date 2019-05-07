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


#include "FrLinearActuatorVelocity.h"


#include "frydom/core/body/FrBody.h"
#include "frydom/core/link/FrLinkBase.h"

#include "frydom/core/math/functions/FrFunctionsInc.h"


namespace frydom {

    namespace internal {

        FrLinkMotorLinearSpeed::FrLinkMotorLinearSpeed(FrLinearActuatorVelocity *frydomActuator) :
                m_frydomActuator(frydomActuator) {
            // No constraints are added by this motor as it is attached to an other link that realizes the link
            SetGuideConstraint(chrono::ChLinkMotorLinear::GuideConstraint::FREE);

        }

        void FrLinkMotorLinearSpeed::SetupInitial() {

            auto frame1 = m_frydomActuator->GetNode1()->GetFrameWRT_COG_InBody();
            frame1.RotY_RADIANS(MU_PI_2,NWU,true);
            auto frame2 = m_frydomActuator->GetNode2()->GetFrameWRT_COG_InBody();
            frame2.RotY_RADIANS(MU_PI_2,NWU,true);

            Initialize(m_frydomActuator->GetChronoBody1(), m_frydomActuator->GetChronoBody2(), true,
                       internal::FrFrame2ChFrame(frame1), internal::FrFrame2ChFrame(frame2));

//            this->Body1 = m_frydomActuator->GetChronoBody1().get();
//            this->Body2 = m_frydomActuator->GetChronoBody2().get();
//
//            this->mask->SetTwoBodiesVariables(&Body1->Variables(), &Body2->Variables());
//
//            this->frame1 = internal::FrFrame2ChFrame(frame1);
//            this->frame2 = internal::FrFrame2ChFrame(frame2);
        }

    } // end namespace internal

    FrLinearActuatorVelocity::FrLinearActuatorVelocity(FrLink *actuatedLink) : FrLinearActuator(actuatedLink) {
        m_chronoActuator = std::make_shared<internal::FrLinkMotorLinearSpeed>(this);
    }

    void FrLinearActuatorVelocity::Initialize() {
        m_chronoActuator->SetupInitial();
    }

    void FrLinearActuatorVelocity::Update(double time) {

    }

    void FrLinearActuatorVelocity::StepFinalize() {

    }

    void FrLinearActuatorVelocity::SetMotorFunction(const FrFunctionBase &function) {
        SetLinearVelocityFunction(function);
    }

    void FrLinearActuatorVelocity::SetLinearVelocityFunction(const FrFunctionBase &function) {

        auto chronoFunctionInterface = internal::FrFunctionChronoInterface(function);

        GetChronoItem_ptr()->SetMotorFunction(chronoFunctionInterface.GetChronoFunction());

    }

    void FrLinearActuatorVelocity::SetConstantLinearVelocity(double velocity) {
        GetChronoItem_ptr()->SetMotorFunction(std::make_shared<chrono::ChFunction_Const>(velocity));
    }

    std::shared_ptr<chrono::ChLink> FrLinearActuatorVelocity::GetChronoLink() {
        return m_chronoActuator;
    }

    internal::FrLinkMotorLinearSpeed *FrLinearActuatorVelocity::GetChronoItem_ptr() const {
        return m_chronoActuator.get();
    }
}  // end namespace frydom
