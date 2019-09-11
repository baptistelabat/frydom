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



#include "FrFlowBase.h"
#include "frydom/core/common/FrFrame.h"
#include "frydom/core/math/functions/ramp/FrCosRampFunction.h"
#include "frydom/environment/FrEnvironment.h"

namespace frydom {

    // -----------------------------------------------------------
    // FLOW BASE
    // -----------------------------------------------------------

    FrFlowBase::FrFlowBase() {
        m_field = std::make_unique<FrUniformField>();
    };

    Velocity FrFlowBase::GetFluxVelocityInWorld(const Position &worldPos, FRAME_CONVENTION fc) const {
        return m_field->GetFluxVelocityInWorld(worldPos, fc) * c_ramp;
    }

    Velocity FrFlowBase::GetRelativeVelocityInFrame(const FrFrame &frame, const Velocity &worldVel,
                                                    FRAME_CONVENTION fc) const {
        Velocity fluxVelocityInWorld = GetFluxVelocityInWorld(frame.GetPosition(fc), fc) - worldVel;
        if (IsNED(fc)) { internal::SwapFrameConvention(fluxVelocityInWorld); }
        return frame.GetQuaternion().GetInverse().Rotate(fluxVelocityInWorld, NWU);
    }

    void FrFlowBase::MakeFieldUniform() {
        NewField<FrUniformField>();
//        m_field = std::make_unique<FrUniformField>();
    }

    FrUniformField* FrFlowBase::GetFieldUniform() const {
        return dynamic_cast<FrUniformField*>(m_field.get());
    }

    void FrFlowBase::Initialize() {
        m_field->Initialize();
        if (GetEnvironment()->GetTimeRamp()->IsActive()) {
            c_ramp = GetEnvironment()->GetTimeRamp()->Get_y(m_time);
        } else {
            c_ramp = 1.;
        }
    }

    void FrFlowBase::Update(double time) {
        m_field->Update(time);
        m_time = time;
    }

    void FrFlowBase::StepFinalize() {
        m_field->StepFinalize();
        if (GetEnvironment()->GetTimeRamp()->IsActive()) {
            c_ramp = GetEnvironment()->GetTimeRamp()->Get_y(m_time);
        } else {
            c_ramp = 1.;
        }
    }

}  // end namespace frydom
