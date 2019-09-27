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
     * \class FrMorisonForce
     * \brief Class for computing Morison loads.
     */
    class FrMorisonForce : public FrForce {


     private:
      std::shared_ptr<FrMorisonElement> m_model;      ///< Morison model linked with the morison force

     public:

      /// Constructor of the morison force with specified morison model
      /// \param model Morison model
      FrMorisonForce(const std::string& name, std::shared_ptr<FrMorisonElement> model);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "MorisonForce"; }

      /// Define a single element morison model
      /// \param body Body to which the morison model is applied
      /// \return Single element morison model
      FrMorisonSingleElement *SetSingleElementModel(FrBody *body);

      /// Define a composite element morison model
      /// \param body Body to which the morison model is applied
      /// \return Composite element morison model
      FrMorisonCompositeElement *SetCompositeElementModel(FrBody *body);

      /// Method to initialized the morison force
      void Initialize() override;

      /// Method to be applied at the end of each time step
      //void StepFinalize() override;

     private:

      /// Compute the Morison force
      /// \param time Current time of the simulation from beginning, in seconds
      void Compute(double time) override;
    };

    /// Maker of a Morison model force : instantiate and return a FrMorisonForce, based on a Morison element.
    /// The makers also add the force to the list of external forces applied on the body.
    /// \param model Morison model, containing the different parameters associated to the model
    /// \param body body on which the force is applied
    /// \return Morison force
    // TODO : delete the body variable, and get it from the node contained in the model?
    std::shared_ptr<FrMorisonForce>
    make_morison_force(const std::string& name,
                       std::shared_ptr<FrMorisonElement> model,
                       std::shared_ptr<FrBody> body);

}  // end namespace frydom

#endif //FRYDOM_FRMORISONFORCE_H



