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


#ifndef FRYDOM_FRWIND_H
#define FRYDOM_FRWIND_H

#include "frydom/core/common/FrObject.h"
#include "frydom/core/common/FrConvention.h"

#include "frydom/environment/flow/FrUniformField.h"  // TODO : include a retirer

#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrUnits.h"


using namespace mathutils;

namespace frydom {

    // ================================================================
    // FrWind : Base wind field
    // ================================================================

    /**
    * \class FrWind
    * \brief Class for defining a wind.
    */
    class FrWind : public FrObject {

    public:

        ~FrWind() = default;

        enum MODEL { UNIFORM };

        virtual chrono::ChVector<> GetFluxVector(FRAME_CONVENTION frame) { }

        virtual void Update(double time) { }

        virtual void Initialize() {}

        virtual void StepFinalize() {}
    };


    // ================================================================
    // FrUniformWind : uniform wind profile class
    // ================================================================

    /**
    * \class FrUniformWind
    * \brief Class for defining a uniform wind.
    */
    class FrUniformWind : virtual public FrWind,
                          virtual public FrUniformField {
    public:
        /// Inheritance of the base constructor
        using FrUniformField::FrUniformField;
    private:
        // Current velocity from FrUniformCurrentField
    public:

        ~FrUniformWind() = default;

        /// Update method from uniform current field class
        void Update(double time) override { FrUniformField::Update(time); }

        /// Get the vector field
        chrono::ChVector<> GetFluxVector(FRAME_CONVENTION frame = NWU) override {
            //return FrUniformField::GetFluxVelocityInWorld(frame);
        }

        /// Method of initialization from uniform current field class
        void Initialize() override { FrUniformField::Initialize(); }

        /// Method of finalize step from uniform current field class
        void StepFinalize() override { FrUniformField::StepFinalize(); }
    };













    // REFACTORING ------------>>>>>>>>>>>>>><

    // -> use FrFlowField

}  // end namespace frydom
#endif //FRYDOM_FRWIND_H
