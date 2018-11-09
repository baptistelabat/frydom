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

#include "frydom/environment/field/FrUniformCurrentField.h"  // TODO : include a retirer

#include "frydom/core/FrVector.h"
#include "frydom/core/FrUnits.h"


// TODO: definir une classe de base pour le champ de courant et de vent (et de houle) afin de ne pas
// repliquer le code concernant la gestion des unites de vitesse, des conventions de direction ("vient de")
// et des reperes d'expression.

using namespace mathutils;

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


//        virtual void Set(chrono::ChVector<>  unit_direction, double  magnitude,
//                         SPEED_UNIT = KNOT, FRAME_CONVENTION= NED,
//                         DIRECTION_CONVENTION convention = GOTO) = 0;

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

//        void Set(chrono::ChVector<>  unit_direction, double  magnitude,
//                 SPEED_UNIT = KNOT, FRAME_CONVENTION= NED,
//                 DIRECTION_CONVENTION convention = GOTO) override;

    };













    /// REFACTORING ------------->>>>>>>>>>>>>>>>

    // Forward declarations
    class FrEnvironment_;
    class FrFrame_;

    // ===============================================================
    // FrCurrent : Base current field
    // ===============================================================

    class FrCurrent_ : public FrObject {

    public:

        ~FrCurrent_() = default;

        virtual Velocity GetWorldFluxVelocity(const Position &worldPos, FRAME_CONVENTION fc) = 0;

        virtual void Update(double time) = 0;

        Velocity GetRelativeVelocityInFrame(const FrFrame_& frame, const Velocity& worldVel, FRAME_CONVENTION fc);

    };

    // ================================================================
    // FrUniformCurrent : uniform current profile class
    // ================================================================

    // Forward declaration
    class FrUniformCurrentField_;

    class FrUniformCurrent_ : public FrCurrent_ {

    private:
        FrEnvironment_* m_environment;
        std::unique_ptr<FrUniformCurrentField_> m_uniformField;

    public:

        explicit FrUniformCurrent_(FrEnvironment_* environment);

        ~FrUniformCurrent_() = default;

        FrUniformCurrentField_* GetField();

        /// Update method from uniform current field class
        void Update(double time) override;

        /// Get the flux velocity at absolute point absPos
        Velocity GetWorldFluxVelocity(const Position &absPos, FRAME_CONVENTION fc) override;

        /// Method of initialization from uniform current field class
        void Initialize() override;

        /// Method of finalize step from uniform current field class
        void StepFinalize() override;

    };









}  // end namespace frydom

#endif //FRYDOM_FRCURRENT_H
