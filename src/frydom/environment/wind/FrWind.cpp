//
// Created by frongere on 03/07/17.
//

#include "FrWind.h"

#include "frydom/environment/FrUniformCurrentField.h"

namespace frydom {

    Velocity FrWind_::GetAbsRelativeVelocity(const Position& absPointPos, const Velocity& absPointVelocity, FRAME_CONVENTION fc) {
        return GetAbsFluxVelocity(absPointPos, fc) - absPointVelocity;
    }

    FrUniformWind_::FrUniformWind_(FrEnvironment_ *environment) : m_environment(environment) {
        m_uniformField = std::make_unique<FrUniformCurrentField_>();
    }

    FrUniformCurrentField_* FrUniformWind_::GetField() {
        return m_uniformField.get();
    }

    void FrUniformWind_::Update(double time) {
        m_uniformField->Update(time);
    }

    Velocity FrUniformWind_::GetAbsFluxVelocity(const Position &absPos, FRAME_CONVENTION fc) {
        return m_uniformField->GetAbsFluxVelocity(fc);
    }

    void FrUniformWind_::Initialize() {
        m_uniformField->Initialize();
    }

    void FrUniformWind_::StepFinalize() {
        m_uniformField->StepFinalize();
    }

}  // end namespace frydom