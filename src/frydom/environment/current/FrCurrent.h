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
#include "frydom/core/FrGeographic.h"
#include "frydom/environment/FrUniformCurrentField.h"
#include "frydom/core/FrVector.h"



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

        virtual chrono::ChVector<> GetFluxVector(FRAME_CONVENTION= NWU) = 0;

        virtual void Update(double time) { }

        virtual void Initialize()  {}

        virtual void StepFinalize()  {}

        virtual void Set(chrono::ChVector<>  unit_direction, double  magnitude,
                         SPEED_UNIT = KNOT, FRAME_CONVENTION= NED,
                         DIRECTION_CONVENTION convention = GOTO) = 0;

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
        chrono::ChVector<> GetFluxVector(FRAME_CONVENTION frame = NWU) override {
            return FrUniformCurrentField::GetFluxVector(frame); }

        /// Method of initialization from uniform current field class
        void Initialize() override { FrUniformCurrentField::Initialize(); }

        /// Method of finalize step from uniform current field class
        void StepFinalize() override { FrUniformCurrentField::StepFinalize(); }

        void Set(chrono::ChVector<>  unit_direction, double  magnitude,
                 SPEED_UNIT = KNOT, FRAME_CONVENTION= NED,
                 DIRECTION_CONVENTION convention = GOTO) override;

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

        virtual Velocity GetFluxVector(const Position& absPos, FRAME_CONVENTION fc) = 0;

        virtual void Update(double time) = 0;

        Velocity GetAbsRelativeVelocity(const Position& absPointPos, const Velocity& absPointVelocity, FRAME_CONVENTION fc);

    };

    // ================================================================
    // FrUniformCurrent : uniform current profile class
    // ================================================================

    class FrUniformCurrent_ : virtual public FrCurrent_,
                              virtual public FrUniformCurrentField_ {
        /// Inheritance of the base constructor
//        using FrUniformCurrentField_::FrUniformCurrentField_;

    private:
        // Current velocity from FrUniformCurrentField

        FrEnvironment_* m_environment;

    public:

        explicit FrUniformCurrent_(FrEnvironment_* environment);

        ~FrUniformCurrent_() = default;

        /// Update method from uniform current field class
        void Update(double time) override;

        /// Get the vector field
        Velocity GetFluxVector(const Position& pos, FRAME_CONVENTION frame = NWU) override;

        /// Method of initialization from uniform current field class
        void Initialize() override;

        /// Method of finalize step from uniform current field class
        void StepFinalize() override;

    };









}  // end namespace frydom

#endif //FRYDOM_FRCURRENT_H
