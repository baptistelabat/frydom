//
// Created by frongere on 08/06/17.
//

#ifndef FRYDOM_FRFORCE_H
#define FRYDOM_FRFORCE_H

#include "frydom/core/FrConstants.h"
#include "chrono/physics/ChForce.h"

#include "FrObject.h"

// Forward declaration
//namespace chrono {
//    class ChForce;
//
//    template <class Real=double>
//    class ChVector;
//}

namespace frydom {

    class FrForce :
            public chrono::ChForce,
            public FrObject
    {

    protected:

        chrono::ChVector<> moment = chrono::VNULL;


    public:

//        FrForce() : moment(chrono::VNULL) {};
//        FrForce() = default;


        /// Updates the time of the object and other stuff that are time-dependent
        /// into the object
        void UpdateTime(double mytime) override {
            ChTime = mytime;

            // ... put time-domain stuff here
        }

        /// Update the force object.
        /// Must be implemented into the child classes.
        virtual void UpdateState() override = 0;

        /// Get the force-torque applied to rigid, body as force vector.
        /// CAUTION !!!! : The force must be returned in the absolute coordinates while the torque must be
        /// expressed in body coordinates
        void GetBodyForceTorque(chrono::ChVector<>& body_force, chrono::ChVector<>& body_torque) const override {
            body_force = force;
            body_torque = moment;
        }

        virtual void Initialize() override {}

        virtual void StepFinalize() override {}

    };








}  // end namespace frydom

#endif //FRYDOM_FRFORCE_H
