//
// Created by frongere on 03/03/2020.
//

#include "frydom/logging/FrTypeNames.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"

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
        force_in_world = -m_line->GetTension(L, NWU);
        relative_position = m_line->GetPositionInWorld(L, NWU);
        break;
    }

    // Set the tension in the world reference frame and NWU frame convention
    SetForceTorqueInWorldAtPointInBody(force_in_world, Torque(), relative_position, NWU);

  }

//  FLUID_TYPE FrCatenaryLineBase::GetFluidType() const {
//    auto environment = GetSystem()->GetEnvironment();
//    auto start_fluid = environment->GetFluidTypeAtPointInWorld(m_startingNode->GetPositionInWorld(NWU), NWU, false);
//    auto end_fluid = environment->GetFluidTypeAtPointInWorld(m_startingNode->GetPositionInWorld(NWU), NWU, false);
//
//    FLUID_TYPE fluidType;
//
//    auto free_surface = GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface();
//    if (start_fluid != end_fluid) {
//      // Determining the absissae of the intersection of the cable with free surface by a bisection algorithm
//      double sa = 0.;
//      double sb = GetUnstretchedLength();
//      while (std::fabs(sb - sa) > 1e-6) {
//
//        double sm = 0.5 * (sa + sb);
//
//        Position Pa = GetPositionInWorld(sa, NWU);
//
//        double da = Pa.z() - free_surface->GetElevation(Pa.x(), Pa.y(), NWU);
//
//        Position Pm = GetPositionInWorld(sm, NWU);
//        double dm = Pm.z() - free_surface->GetElevation(Pm.x(), Pm.y(), NWU);
//
//        if (da * dm <= 0.) {
//          sb = sm;
//        } else {
//          sa = sm;
//        }
//      }
//
//      fluidType = (sa > 0.5 * GetUnstretchedLength()) ? start_fluid : end_fluid;
//
//    } else {
//      fluidType = start_fluid;
//    }
//
//    return start_fluid;
//  }


}  // end namespace frydom
