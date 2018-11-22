//
// Created by frongere on 03/07/17.
//

#ifndef FRYDOM_FRWIND_H
#define FRYDOM_FRWIND_H

#include "frydom/core/FrObject.h"
#include "frydom/core/FrConvention.h"

#include "frydom/environment/flow/FrUniformField.h"  // TODO : include a retirer

#include "frydom/core/FrVector.h"
#include "frydom/core/FrUnits.h"


using namespace mathutils;

namespace frydom {

    // ================================================================
    // FrWind : Base wind field
    // ================================================================

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













    /// REFACTORING ------------>>>>>>>>>>>>>><

    /// -> use FrFlowField

}  // end namespace frydom
#endif //FRYDOM_FRWIND_H
