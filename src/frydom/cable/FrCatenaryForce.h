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
   * \class FrCatenaryForce FrCatenaryForce.h
   * \brief Class for getting the tension from the catenary line, subclass of FrForce.
   * This class get the tension computed by the catenary line class, to the body on which the force is applied.
   * A differenciation is done on which side of the cable (starting or ending), the force is applied.
   * \see FrCatenaryLine_, FrForce
   */
  class FrCatenaryForce : public FrForce {

   private:

    FrCatenaryLine *m_line; ///< The parent line
    FrCatenaryLine::LINE_SIDE m_line_side;   ///< The side of the line where the tension is applied

   public:

    /// Get the type name of this object
    /// \return type name of this object
    std::string GetTypeName() const override { return "CatenaryForce"; }

    /// FrCatenaryForce constructor, from a catenary line, and the description of the side of this line
    /// \param line catenary line applying a tension
    /// \param side side of the line (starting or ending)
    FrCatenaryForce(const std::string &name,
                    FrBody *body,
                    FrCatenaryLine *line,
                    FrCatenaryLine::LINE_SIDE side) :
        FrForce(name, body),
        m_line(line),
        m_line_side(side) {};

    /// Return true if the force is included in the static analysis
    bool IncludedInStaticAnalysis() const override { return true; }

   private:

    /// Update the catenary force : get the tension applied by the line on the corresponding node
    /// \param time time of the simulation
    void Compute(double time) override;

  };

}  // end namespace frydom


#endif //FRYDOM_FRCATENARYFORCE_H
