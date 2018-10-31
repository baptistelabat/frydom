// =============================================================================
// PROJECT FRyDoM
//
// Copyright (c) 2017 Ecole Centrale de Nantes
// All right reserved.
//
//
// =============================================================================
// Authors: Francois Rongere
// =============================================================================
//
// Base for marine current modeling
//
// =============================================================================


#include "FrCurrent.h"

#include "frydom/environment/field/FrUniformCurrentField.h"


namespace frydom {

    Velocity FrCurrent_::GetAbsRelativeVelocity(const Position& absPointPos, const Velocity& absPointVelocity, FRAME_CONVENTION fc) {
        return GetAbsFluxVelocity(absPointPos, fc) - absPointVelocity;
    }

    FrUniformCurrent_::FrUniformCurrent_(FrEnvironment_ *environment) : m_environment(environment) {
        m_uniformField = std::make_unique<FrUniformCurrentField_>();
    }

    FrUniformCurrentField_* FrUniformCurrent_::GetField() {
        return m_uniformField.get();
    }

    void FrUniformCurrent_::Update(double time) {
        m_uniformField->Update(time);
    }

    Velocity FrUniformCurrent_::GetAbsFluxVelocity(const Position &absPos, FRAME_CONVENTION fc) {
        return m_uniformField->GetAbsFluxVelocity(fc);
    }

    void FrUniformCurrent_::Initialize() {
        m_uniformField->Initialize();
    }

    void FrUniformCurrent_::StepFinalize() {
        m_uniformField->StepFinalize();
    }

}  // end namespace frydom
