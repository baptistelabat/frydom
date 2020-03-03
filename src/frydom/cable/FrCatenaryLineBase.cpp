//
// Created by frongere on 03/03/2020.
//

#include <frydom/logging/FrTypeNames.h>
#include "FrCatenaryLineBase.h"


namespace frydom {




  FrCatenaryForce::FrCatenaryForce(const std::string &name,
                                   FrBody *body,
                                   FrCatenaryLineBase *line,
                                   FrCatenaryLineBase::LINE_SIDE side) :
      FrForce(name, TypeToString(this), body),
      m_line(line),
      m_line_side(side) {}

  bool FrCatenaryForce::IncludedInStaticAnalysis() const { return true; }

  void FrCatenaryForce::Compute(double time) {

    Position relative_position;
    Force force_in_world;

    // Get the line tension from the corresponding node
    switch (m_line_side) {
      case FrCatenaryLineBase::LINE_START:
        force_in_world = m_line->GetTension(0., NWU);
        relative_position = m_line->GetPositionInWorld(0., NWU);
        break;

      case FrCatenaryLineBase::LINE_END:
        double L = m_line->GetUnstretchedLength();
        force_in_world = - m_line->GetTension(L, NWU);
        relative_position = m_line->GetPositionInWorld(L, NWU);
        break;
    }

    // Set the tension in the world reference frame and NWU frame convention
    SetForceTorqueInWorldAtPointInBody(force_in_world, Torque(), relative_position, NWU);

  }

}  // end namespace frydom
