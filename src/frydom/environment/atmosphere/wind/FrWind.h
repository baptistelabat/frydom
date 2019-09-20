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


#ifndef FRYDOM_FRWIND_H
#define FRYDOM_FRWIND_H

#include "frydom/environment/flow/FrFlowBase.h"


namespace frydom {

    // Forward declarations
    template<typename OffshoreSystemType>
    class FrEnvironment;

    template<typename OffshoreSystemType>
    class FrAtmosphere;

    /**
    * \class FrWind
    * \brief Class for defining a wind.
    */
    template<typename OffshoreSystemType>
    class FrWind : public FrFlowBase<OffshoreSystemType> {
     private:

      FrAtmosphere<OffshoreSystemType> *m_atmosphere;  ///> Pointer to the atmosphere containing this wind model

     public:

      /// Default constructor
      /// \param atmosphere containing this wind model
      explicit FrWind(FrAtmosphere<OffshoreSystemType> *atmosphere)
          : FrFlowBase<OffshoreSystemType>() { m_atmosphere = atmosphere; }

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "Wind"; }

      /// Get the atmosphere containing this wind model
      /// \return atmosphere containing this wind model
      FrAtmosphere<OffshoreSystemType> *GetAtmosphere() const { return m_atmosphere; }

      FrEnvironment<OffshoreSystemType> *GetEnvironment() const override;

    };

}  // end namespace frydom

#endif //FRYDOM_FRWIND_H
