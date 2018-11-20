//
// Created by camille on 17/04/18.
//

#ifndef FRYDOM_FRMORISONFORCE_H
#define FRYDOM_FRMORISONFORCE_H


#include "frydom/core/FrForce.h"

#include "FrMorisonModel.h"


namespace frydom {

    class FrMorisonForce : public FrForce {

    private:
        FrMorisonModel* m_element;

    public:

        /// Default constructor of the morison force
        FrMorisonForce();;

        /// Constructor with definition of the morison model
        FrMorisonForce(FrMorisonModel* element);

        /// Definition of the model
        void SetElement(FrMorisonModel* element);

        /// Update of the morison model
        void UpdateState() override;

        void UpdateTime(const double time) override;

        void Update(const double time) override;

        /// Initialize the morison elements
        void Initialize() override;

        /// Definition of the prefix used in log file
        void SetLogPrefix(std::string prefix_name) override;

        /// Apply an external force to the body
        void SetBodyForce(chrono::ChVector<> mforce);

        /// Return the force applied to the body
        chrono::ChVector<double> GetBodyForce() const;

        /// Apply an external moment to the body
        void SetBodyTorque(chrono::ChVector<> torque);

        /// Return the moment applied to the body
        chrono::ChVector<double> GetBodyTorque() const;

    };

}

#endif //FRYDOM_FRMORISONFORCE_H



