//
// Created by frongere on 03/07/17.
//

#include "FrWind.h"

namespace frydom {


    FrWind_::~FrWind_() = default;

    FrUniformWind_::FrUniformWind_(FrEnvironment_ *environment) : m_environment(environment) {}

    FrUniformWind_::~FrUniformWind_() = default;

    void FrUniformWind_::Update(double time) {
        FrUniformCurrentField::Update(time);
    }

    chrono::ChVector<> FrUniformWind_::GetFluxVector(FRAME_CONVENTION frame) {
        return FrUniformCurrentField::GetFluxVector(frame);
    }

    void FrUniformWind_::Initialize() {
        FrUniformCurrentField::Initialize();
    }

    void FrUniformWind_::StepFinalize() {
        FrUniformCurrentField::StepFinalize();
    }


}  // end namespace frydom