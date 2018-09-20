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


namespace frydom {


    FrUniformCurrent_::FrUniformCurrent_(FrEnvironment_ *environment) : m_environment(environment){

    }

    void FrUniformCurrent_::Update(double time) {
        FrUniformCurrentField::Update(time);
    }

    chrono::ChVector<> FrUniformCurrent_::GetFluxVector(FrFrame frame) {
        return FrUniformCurrentField::GetFluxVector(frame);
    }

    void FrUniformCurrent_::Initialize() {
        FrUniformCurrentField::Initialize();
    }

    void FrUniformCurrent_::StepFinalize() {
        FrUniformCurrentField::StepFinalize();
    }

    void FrUniformCurrent::Set(chrono::ChVector<>  unit_direction, double  magnitude,
                               SPEED_UNIT unit, FrFrame frame,
                               FrDirectionConvention convention) {
        FrUniformCurrentField::Set(unit_direction, magnitude, unit, frame, convention);
    }

}  // end namespace frydom
