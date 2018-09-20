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
// Base for marine current modeling.
//
// =============================================================================

#ifndef FRYDOM_FRCURRENT_H
#define FRYDOM_FRCURRENT_H

#include "frydom/core/FrObject.h"
#include "frydom/core/FrConstants.h"
#include "frydom/environment/FrUniformCurrentField.h"



// TODO: definir une classe de base pour le champ de courant et de vent (et de houle) afin de ne pas
// repliquer le code concernant la gestion des unites de vitesse, des conventions de direction ("vient de")
// et des reperes d'expression.


namespace frydom {

    // ===============================================================
    // FrCurrent : Base current field
    // ===============================================================

    class FrCurrent : public FrObject {

    public:

        ~FrCurrent() = default;

        enum MODEL { UNIFORM };

        virtual chrono::ChVector<> GetFluxVector(FrFrame= NWU) { }

        virtual void Update(double time) { }

        virtual void Initialize()  {}

        virtual void StepFinalize()  {}

        virtual void Set(chrono::ChVector<>  unit_direction, double  magnitude,
                         SPEED_UNIT = KNOT, FrFrame= NED,
                         FrDirectionConvention convention = GOTO) = 0;




    };

    // ================================================================
    // FrUniformCurrent : uniform current profile class
    // ================================================================

    class FrUniformCurrent : virtual public FrCurrent,
                             virtual public FrUniformCurrentField {
        /// Inheritance of the base constructor
        using FrUniformCurrentField::FrUniformCurrentField;

    private:
        // Current velocity from FrUniformCurrentField
    public:

        ~FrUniformCurrent() = default;

        /// Update method from uniform current field class
        void Update(double time) override { FrUniformCurrentField::Update(time);}

        /// Get the vector field
        chrono::ChVector<> GetFluxVector(FrFrame frame = NWU) override {
            return FrUniformCurrentField::GetFluxVector(frame); }

        /// Method of initialization from uniform current field class
        void Initialize() override { FrUniformCurrentField::Initialize(); }

        /// Method of finalize step from uniform current field class
        void StepFinalize() override { FrUniformCurrentField::StepFinalize(); }

        void Set(chrono::ChVector<>  unit_direction, double  magnitude,
                 SPEED_UNIT = KNOT, FrFrame= NED,
                 FrDirectionConvention convention = GOTO) override;

    };













    /// REFACTORING ------------->>>>>>>>>>>>>>>>

    // Forward declarations
    class FrEnvironment_;

    // ===============================================================
    // FrCurrent : Base current field
    // ===============================================================

    class FrCurrent_ : public FrObject {

    public:

        ~FrCurrent_() = default;

        enum MODEL { UNIFORM };

        virtual chrono::ChVector<> GetFluxVector(FrFrame= NWU) = 0;

        virtual void Update(double time) = 0;

//        virtual void Initialize()  {}
//
//        virtual void StepFinalize()  {}

//        virtual void Set(chrono::ChVector<>  unit_direction, double  magnitude,
//                         SPEED_UNIT = KNOT, FrFrame= NED,
//                         FrDirectionConvention convention = GOTO) = 0;




    };

    // ================================================================
    // FrUniformCurrent : uniform current profile class
    // ================================================================

    class FrUniformCurrent_ : virtual public FrCurrent_,
                             virtual public FrUniformCurrentField {
        /// Inheritance of the base constructor
        using FrUniformCurrentField::FrUniformCurrentField;

    private:
        // Current velocity from FrUniformCurrentField

        FrEnvironment_* m_environment;

    public:

        explicit FrUniformCurrent_(FrEnvironment_* environment);

        ~FrUniformCurrent_() = default;

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

#endif //FRYDOM_FRCURRENT_H
