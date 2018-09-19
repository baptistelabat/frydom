//
// Created by frongere on 03/07/17.
//

#ifndef FRYDOM_FRWIND_H
#define FRYDOM_FRWIND_H

#include "chrono/core/ChVector.h"

#include "frydom/core/FrObject.h"
#include "frydom/core/FrConstants.h"
#include "frydom/environment/FrConventions.h"
#include "frydom/environment/FrUniformCurrentField.h"

#include "MathUtils/MathUtils.h"


using namespace mathutils;


namespace frydom {

    // ================================================================
    // FrWind : Base wind field
    // ================================================================

    class FrWind : public FrObject {

    public:

        enum MODEL { UNIFORM };

        virtual chrono::ChVector<> GetFluxVector(FrFrame frame) { }

        virtual void Update(double time) { }

        virtual void Initialize() {}

        virtual void StepFinalize() {}
    };


    // ================================================================
    // FrUniformWind : uniform wind profile class
    // ================================================================

    class FrUniformWind : virtual public FrWind,
                          virtual public FrUniformCurrentField {
    public:
        /// Inheritance of the base constructor
        using FrUniformCurrentField::FrUniformCurrentField;
    private:
        // Current velocity from FrUniformCurrentField
    public:

        /// Update method from uniform current field class
        void Update(double time) override { FrUniformCurrentField::Update(time); }

        /// Get the vector field
        chrono::ChVector<> GetFluxVector(FrFrame frame = NWU) override {
            return FrUniformCurrentField::GetFluxVector(frame); }

        /// Method of initialization from uniform current field class
        void Initialize() override { FrUniformCurrentField::Initialize(); }

        /// Method of finalize step from uniform current field class
        void StepFinalize() override { FrUniformCurrentField::StepFinalize(); }
    };













    /// REFACTORING ------------>>>>>>>>>>>>>><

    class FrEnvironment_;


    // ================================================================
    // FrWind : Base wind field
    // ================================================================

    class FrWind_ : public FrObject {

    public:

        enum MODEL { UNIFORM };

        virtual chrono::ChVector<> GetFluxVector(FrFrame frame=NWU) = 0;

        virtual void Update(double time) = 0;

//        virtual void Initialize() {}
//
//        virtual void StepFinalize() {}
    };


    // ================================================================
    // FrUniformWind : uniform wind profile class
    // ================================================================

    class FrUniformWind_ : virtual public FrWind_,
                          virtual public FrUniformCurrentField {
    public:
        /// Inheritance of the base constructor
        using FrUniformCurrentField::FrUniformCurrentField;

    private:
        // Current velocity from FrUniformCurrentField

        FrEnvironment_* m_environment;

    public:

        explicit FrUniformWind_(FrEnvironment_* environment);

        /// Update method from uniform current field class
        void Update(double time) override;

        /// Get the vector field
        chrono::ChVector<> GetFluxVector(FrFrame frame = NWU) override;

        /// Method of initialization from uniform current field class
        void Initialize() override;

        /// Method of finalize step from uniform current field class
        void StepFinalize() override;
    };

}  // end namespace frydom
#endif //FRYDOM_FRWIND_H
