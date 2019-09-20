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
    template<typename OffshoreSystemType>
    FrFlowBase<OffshoreSystemType>::FrFlowBase() {
      m_field = std::make_unique<FrUniformField<OffshoreSystemType>>();
    };

    template<typename OffshoreSystemType>
    Velocity FrFlowBase<OffshoreSystemType>::GetFluxVelocityInWorld(const Position &worldPos, FRAME_CONVENTION fc) const {
      return m_field->GetFluxVelocityInWorld(worldPos, fc) * c_ramp;
    }

    template<typename OffshoreSystemType>
    Velocity FrFlowBase<OffshoreSystemType>::GetRelativeVelocityInFrame(const FrFrame &frame, const Velocity &worldVel,
                                                    FRAME_CONVENTION fc) const {
      Velocity fluxVelocityInWorld = GetFluxVelocityInWorld(frame.GetPosition(fc), fc) - worldVel;
      if (IsNED(fc)) { internal::SwapFrameConvention(fluxVelocityInWorld); }
      return frame.GetQuaternion().GetInverse().Rotate(fluxVelocityInWorld, NWU);
    }

    template <class OffshoreSystemType>
    template<class T>
    void FrFlowBase<OffshoreSystemType>::NewField() {
      m_field = std::make_unique<T>();
    }


    template<typename OffshoreSystemType>
    void FrFlowBase<OffshoreSystemType>::MakeFieldUniform() {
      m_field = std::make_unique<FrUniformField<OffshoreSystemType>>();
    }

    template <class OffshoreSystemType>
    template<class T>
    T *FrFlowBase<OffshoreSystemType>::GetField() const {
      return dynamic_cast<T *>(m_field.get());
    }

    template<typename OffshoreSystemType>
    FrUniformField<OffshoreSystemType> *FrFlowBase<OffshoreSystemType>::GetFieldUniform() const {
      return dynamic_cast<FrUniformField<OffshoreSystemType> *>(m_field.get());
    }

    template<typename OffshoreSystemType>
    void FrFlowBase<OffshoreSystemType>::Initialize() {
      m_field->Initialize();
      if (GetEnvironment()->GetTimeRamp()->IsActive()) {
        c_ramp = GetEnvironment()->GetTimeRamp()->Get_y(m_time);
      } else {
        c_ramp = 1.;
      }
    }

    template<typename OffshoreSystemType>
    void FrFlowBase<OffshoreSystemType>::Update(double time) {
      m_field->Update(time);
      m_time = time;
    }

    template<typename OffshoreSystemType>
    void FrFlowBase<OffshoreSystemType>::StepFinalize() {
      m_field->StepFinalize();
      if (GetEnvironment()->GetTimeRamp()->IsActive()) {
        c_ramp = GetEnvironment()->GetTimeRamp()->Get_y(m_time);
      } else {
        c_ramp = 1.;
      }
    }

}  // end namespace frydom
