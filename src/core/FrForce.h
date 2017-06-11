//
// Created by frongere on 08/06/17.
//

#ifndef FRYDOM_FRFORCE_H
#define FRYDOM_FRFORCE_H


#include "chrono/physics/ChForce.h"

namespace frydom {

    class FrForce : public chrono::ChForce {

    private:
        bool useChronoModel;  ///< Do we use the native chrono model for force in addition to our model.

    public:
        // TODO: utilser modula pour moduler les composantes des forces dans les nouveaux modeles.

        /// Constructor.
        /// By default, the useChronoModel is set to true. It may change in the future.
        FrForce() : useChronoModel(false), chrono::ChForce() {};

        /// Update the state of the force.
        /// This is where the force has to be calculated based on time, body's state, environment, etc.
        /// This method totally overrides the chrono base class method as it does not correspond to the
        /// type of force we usaually use.
        /// Note however that every subclass should reimplement this method.
        virtual void UpdateState() override {
            force.SetNull();  // Important as UpdateForce
            UpdateApplicationPoint();
            if (useChronoModel) {
                UpdateChronoForce(); // allows to keep using what chrono defined as force model
            }
            UpdateForce();

        };

        /// Updates the application point
        virtual void UpdateApplicationPoint();

        /// Update the force in the chrono model
        virtual void UpdateChronoForce();

        /// Update the properly tuned force model
        virtual void UpdateForce() = 0;

    };

}  // end namespace frydom

#endif //FRYDOM_FRFORCE_H
