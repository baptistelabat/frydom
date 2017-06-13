//
// Created by frongere on 08/06/17.
//

#ifndef FRYDOM_FRFORCE_H
#define FRYDOM_FRFORCE_H


#include "chrono/physics/ChForce.h"

namespace frydom {

    class FrForce : public chrono::ChForce {

    protected:
//        ReferenceFrame frame;  ///< fix position in body csys or world csys
//
//        chrono::ChVector<> vpoint;     ///< absolute point of application
//        chrono::ChVector<> vrelpoint;  ///< relative point of application
//
//        chrono::ChVector<> force;
        chrono::ChVector<> moment;

    public:

        FrForce() : chrono::ChForce(),
                    moment(chrono::VNULL) {};

        /// Updates the time of the object and other stuff that are time-dependent
        /// into the object
        virtual void UpdateTime(double mytime) override {
            ChTime = mytime;

            // ... put time-domain stuff here
        }

        /// Update the force object.
        /// Must be implemented into the child classes.
        virtual void UpdateState() override = 0;

        /// Get the force-torque applied to rigid, body as force vector.
        /// The force must be returned in the absolute coordinates while the torque must be
        /// expressed in body coordinates
        virtual void GetBodyForceTorque(chrono::ChVector<>& body_force, chrono::ChVector<>& body_torque) const override {
            body_force = force;
            body_torque = moment;
        }



    private:
//        // Making unused function private for them not to be accessible into child classes
//        void SetMode(ForceType m_mode) override { mode = m_mode; }
//        ForceType GetMode() const override { return mode; }
//
//        void SetAlign(AlignmentFrame m_align) override { align = m_align; }
//        AlignmentFrame GetAlign() const override { return align; }
//
//        chrono::ChVector<> GetDir() const { return vdir; }
//        chrono::ChVector<> GetRelDir() const { return vreldir; }
//
//        void SetDir(chrono::ChVector<> newf) override {};
//        void SetRelDir(chrono::ChVector<> newf) override {};
//
//        virtual void SetMforce(double newf) override {};
//        virtual double GetMforce() const override { return mforce; }

    };

}  // end namespace frydom

#endif //FRYDOM_FRFORCE_H
