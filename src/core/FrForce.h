//
// Created by frongere on 08/06/17.
//

#ifndef FRYDOM_FRFORCE_H
#define FRYDOM_FRFORCE_H


#include "chrono/physics/ChForce.h"

namespace frydom {

    class FrForce : public chrono::ChForce {

    protected:
        bool useChronoModel;  ///< Do we use the native chrono model for force in addition to our model.

    public:


    private:
        // Making unused function private for them not to be accessible into child classes
        void SetMode(ForceType m_mode) override { mode = m_mode; }
        ForceType GetMode() const override { return mode; }

        void SetAlign(AlignmentFrame m_align) override { align = m_align; }
        AlignmentFrame GetAlign() const override { return align; }

        chrono::ChVector<> GetDir() const { return vdir; }
        chrono::ChVector<> GetRelDir() const { return vreldir; }

        void SetDir(chrono::ChVector<> newf) override {};
        void SetRelDir(chrono::ChVector<> newf) override {};

        virtual void SetMforce(double newf) override {};
        virtual double GetMforce() const override { return mforce; }




//        // TODO: utilser modula pour moduler les composantes des forces dans les nouveaux modeles.
//
//        /// Constructor.
//        /// By default, the useChronoModel is set to true. It may change in the future.
//        FrForce() : useChronoModel(false), chrono::ChForce() {};
//
//        /// Update the state of the force.
//        /// This is where the force has to be calculated based on time, body's state, environment, etc.
//        /// This method totally overrides the chrono base class method as it does not correspond to the
//        /// type of force we usaually use.
//        /// Note however that every subclass should reimplement this method.
//        virtual void UpdateState() override {
//            force.SetNull();  // Important as UpdateForce
//            UpdateApplicationPoint();
//            if (useChronoModel) {
//                UpdateChronoForce(); // allows to keep using what chrono defined as force model
//            }
//            UpdateForce();
//
//        };

        /// Updates the application point
        virtual void UpdateApplicationPoint();

        /// Update the force in the chrono model
        virtual void UpdateChronoForce();

        /// Update the properly tuned force model
        virtual void UpdateForce() = 0;

    };

}  // end namespace frydom

#endif //FRYDOM_FRFORCE_H
