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


#ifndef FRYDOM_FRCATENARYFORCE_H
#define FRYDOM_FRCATENARYFORCE_H

#include "frydom/core/force/FrForce.h"
#include "frydom/cable/FrCatenaryLine.h"

namespace frydom {

    /**
     * \class FrCatenaryForce_ FrCatenaryForce.h
     * \brief Class for getting the tension from the catenary line, subclass of FrForce_.
     * This class get the tension computed by the catenary line class, to the body on which the force is applied.
     * A differenciation is done on which side of the cable (starting or ending), the force is applied.
     * \see FrCatenaryLine_, FrForce_
     */
    class FrCatenaryForce_ : public FrForce_ {

    private:

        FrCatenaryLine_* m_line; ///< The parent line
        FrCatenaryLine_::LINE_SIDE m_line_side;   ///< The side of the line where the tension is applied

    public:

        /// FrCatenaryForce constructor, from a catenary line, and the description of the side of this line
        /// \param line catenary line applying a tension
        /// \param side side of the line (starting or ending)
        FrCatenaryForce_(FrCatenaryLine_* line, FrCatenaryLine_::LINE_SIDE side) : m_line(line), m_line_side(side) {};

        /// Update the catenary force : get the tension applied by the line on the corresponding node
        /// \param time time of the simulation
        void Update(double time) override;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;;

    };

}  // end namespace frydom


#endif //FRYDOM_FRCATENARYFORCE_H
