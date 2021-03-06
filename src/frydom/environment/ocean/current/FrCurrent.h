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


#ifndef FRYDOM_FRCURRENT_H
#define FRYDOM_FRCURRENT_H

#include "frydom/environment/flow/FrFlowBase.h"


// TODO: definir une classe de base pour le champ de courant et de vent (et de houle) afin de ne pas
// repliquer le code concernant la gestion des unites de vitesse, des conventions de direction ("vient de")
// et des reperes d'expression.


namespace frydom {

  // Forward declaration
  class FrOcean;


  /**
  * \class FrCurrent
  * \brief Class defining a current field.
  */
  class FrCurrent : public FrFlowBase {
   private:

    FrOcean *m_ocean;  ///> Pointer to the ocean containing this current model

   public:
    /// Default constructor
    /// \param ocean ocean containing this current model
    explicit FrCurrent(FrOcean *ocean) : FrFlowBase() { m_ocean = ocean; }

    /// Get the ocean containing this current model
    /// \return ocean containing this current model
    FrOcean *GetOcean() const { return m_ocean; }

    FrEnvironment *GetEnvironment() const override;

  };

}  // end namespace frydom

#endif //FRYDOM_FRCURRENT_H
