// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#ifndef FRYDOM_FRMORISONFORCE_H
#define FRYDOM_FRMORISONFORCE_H

#include "frydom/core/force/FrForce.h"

namespace frydom {


//    class FrMorisonModel;
//
//    /**
//     * \class FrMorisonForce
//     * \brief Class for computing Morison loads.
//     */
//    class FrMorisonForce : public FrForce {
//
//    private:
//        FrMorisonModel* m_element;
//
//    public:
//
//        /// Default constructor of the morison force
//        FrMorisonForce();;
//
//        /// Constructor with definition of the morison model
//        FrMorisonForce(FrMorisonModel* element);
//
//        /// Definition of the model
//        void SetElement(FrMorisonModel* element);
//
//        /// Update of the morison model
//        void UpdateState() override;
//
//        void UpdateTime(const double time) override;
//
//        void Update(const double time) override;
//
//        /// Initialize the morison elements
//        void Initialize() override;
//
//        /// Definition of the prefix used in log file
//        void SetLogPrefix(std::string prefix_name) override;
//
//        /// Apply an external force to the body
//        void SetBodyForce(chrono::ChVector<> mforce);
//
//        /// Return the force applied to the body
//        chrono::ChVector<double> GetBodyForce() const;
//
//        /// Apply an external moment to the body
//        void SetBodyTorque(chrono::ChVector<> torque);
//
//        /// Return the moment applied to the body
//        chrono::ChVector<double> GetBodyTorque() const;
//
//    };
//
//
//
//
//
//
//
//
//
//
//    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<< REFACTORING

    class FrMorisonElement_;
    class FrMorisonSingleElement_;
    class FrMorisonCompositeElement_;


    /**
     * \class FrMorisonForce_
     * \brief Class for computing Morison loads.
     */
    class FrMorisonForce_ : public FrForce_ {


    private:
        std::shared_ptr<FrMorisonElement_> m_model;

    public:

        FrMorisonForce_(std::shared_ptr<FrMorisonElement_> model)
            : m_model(model) { }

        FrMorisonSingleElement_* SetSingleElementModel(FrBody_* body);

        FrMorisonCompositeElement_* SetCompositeElementModel(FrBody_* body);

        void Update(double time) override;

        void Initialize() override;

        void StepFinalize() override;
    };

    /// Maker of a Morison model force : instantiate and return a FrMorisonForce, based on a Morison element.
    /// The makers also add the force to the list of external forces applied on the body.
    /// \param model Morison model, containing the different parameters associated to the model
    /// \param body body on which the force is applied
    /// \return Morison force
    // TODO : delete the body variable, and get it from the node contained in the model?
    std::shared_ptr<FrMorisonForce_>
    make_morison_force(std::shared_ptr<FrMorisonElement_> model, std::shared_ptr<FrBody_> body);

}

#endif //FRYDOM_FRMORISONFORCE_H



