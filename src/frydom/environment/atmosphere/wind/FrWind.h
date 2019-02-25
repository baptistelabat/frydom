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

//#include "frydom/core/common/FrObject.h"
//#include "frydom/core/common/FrConvention.h"
//
//#include "frydom/environment/flow/FrUniformField.h"  // TODO : include a retirer
//
//#include "frydom/core/math/FrVector.h"
//#include "frydom/core/common/FrUnits.h"


#include "frydom/environment/flow/FrFlowBase.h"


using namespace mathutils;

namespace frydom {

    class FrAtmosphere_;

    /**
    * \class FrWind_
    * \brief Class for defining a wind.
    */
    class FrWind_ : public FrFlowBase {
    private:

        FrAtmosphere_* m_atmosphere;  ///> Pointer to the atmosphere containing this wind model

    public:

        /// Default constructor
        /// \param atmosphere containing this wind model
        explicit FrWind_(FrAtmosphere_* atmosphere) : FrFlowBase() { m_atmosphere = atmosphere;}

        /// Get the atmosphere containing this wind model
        /// \return atmosphere containing this wind model
        FrAtmosphere_* GetAtmosphere() const {return m_atmosphere;}

        FrEnvironment_* GetEnvironment() const override;

    };

}  // end namespace frydom

#endif //FRYDOM_FRWIND_H
