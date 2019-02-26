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

#include <memory>

#include "frydom/core/force/FrForce.h"



namespace frydom {

    // Forward declarations
    class FrMorisonElement;
    class FrMorisonSingleElement;
    class FrMorisonCompositeElement;


    /**
     * \class FrMorisonForce_
     * \brief Class for computing Morison loads.
     */
    class FrMorisonForce : public FrForce {


    private:
        std::shared_ptr<FrMorisonElement> m_model;

    public:

        explicit FrMorisonForce(std::shared_ptr<FrMorisonElement> model)
            : m_model(model) { }

        FrMorisonSingleElement* SetSingleElementModel(FrBody* body);

        FrMorisonCompositeElement* SetCompositeElementModel(FrBody* body);

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
    std::shared_ptr<FrMorisonForce>
    make_morison_force(std::shared_ptr<FrMorisonElement> model, std::shared_ptr<FrBody> body);

}  // end namespace frydom

#endif //FRYDOM_FRMORISONFORCE_H



