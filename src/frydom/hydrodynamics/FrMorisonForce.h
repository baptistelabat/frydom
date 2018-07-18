//
// Created by camille on 17/04/18.
//

#ifndef FRYDOM_FRMORISONFORCE_H
#define FRYDOM_FRMORISONFORCE_H

#include "frydom/hydrodynamics/FrMorisonModel.h"
#include "frydom/core/FrForce.h"

namespace frydom {

    class FrMorisonForce : public FrForce {

    private:
        FrMorisonModel* m_element;

    public:

        /// Default constructor of the morison force
        FrMorisonForce() : m_element(NULL) {};

        /// Constructor with definition of the morison model
        FrMorisonForce(FrMorisonModel* element) {
            m_element = element;
        }

        /// Definition of the model
        void SetElement(FrMorisonModel* element) {
            m_element = element;
        }

        /// Update of the morison model
        void UpdateState() override {
            m_element->UpdateState();
        }

        void UpdateTime(const double time) override {
            ChTime = time;
        }

        void Update(const double time) override {
            UpdateTime(time);
            UpdateState();
        }

        /// Initialize the morison elements
        void Initialize() override {
            m_element->Initialize();
        }

        /// Apply an external force to the body
        void SetBodyForce(chrono::ChVector<> mforce) { force = mforce; }

        /// Return the force applied to the body
        chrono::ChVector<double> GetBodyForce() const { return force; }

        /// Apply an external moment to the body
        void SetBodyTorque(chrono::ChVector<> torque) { moment = torque; }

        /// Return the moment applied to the body
        chrono::ChVector<double> GetBodyTorque() const { return moment; }

    };

}

#endif //FRYDOM_FRMORISONFORCE_H


