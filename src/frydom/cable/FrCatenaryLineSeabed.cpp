//
// Created by frongere on 27/02/2020.
//

#include "FrCatenaryLineSeabed.h"


#include "frydom/logging/FrTypeNames.h"
#include "frydom/logging/FrEventLogger.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/seabed/FrSeabed.h"

namespace frydom {


  FrCatenaryLineSeabed::FrCatenaryLineSeabed(const std::string &name,
                                             const std::shared_ptr<FrNode> &anchorNode,
                                             const std::shared_ptr<FrNode> &fairleadNode,
                                             const std::shared_ptr<FrCableProperties> &properties,
                                             bool elastic,
                                             double unstretchedLength,
                                             FLUID_TYPE fluid) :
      FrCatenaryLine(name,
                     GetSystem()->GetWorldBody()->NewNode(name + "_TDP_node"),
                     fairleadNode,
                     properties,
                     elastic,
                     unstretchedLength,
                     fluid),
      m_anchor_node(anchorNode),
      m_Lb(0.),
      c_Cb(1.),
      m_use_seabed_interaction_solver(false) {

  }

  void FrCatenaryLineSeabed::Initialize() {
    // TODO: tester que anchor est bien sur le seabed...

    // Getting the cable seabed friction coeffient from seabed ??? pas de definition par ligne ???
    c_Cb = 1.; // TODO: aller chercher le coeff dans FrSeabed !!! Constant par la suite donc on le met en cache

    // Initially, we put the TDP at anchor position
    GetTouchDownPointNode()->SetPositionInWorld(m_anchor_node->GetPositionInWorld(NWU), NWU);

    FrCatenaryLine::Initialize();

    // Now we make a first guess on the TDP position by searching the intersection with the seabed
    m_Lb = GetSeabedIntersection();

    // Getting the direction of the lying part of the line on seabed (intersection of the line plane and the seabed plane)
    m_lying_direction = (GetFairleadNode()->GetPositionInWorld(NWU) -
                         GetAnchorNode()->GetPositionInWorld(NWU)).normalized();
    m_lying_direction = m_lying_direction.cross(m_u);
    m_lying_direction = m_lying_direction.cross(Direction(0, 0, 1));

    // TDP node update
    UpdateTouchDownPointPosition();
    m_unstretchedLength -= m_Lb;

    // Updating the catenary (to get also a first rough estimate of the tension at tdp)
    solve();

    // Now we switch to the solver that deal with seabed interaction
    m_use_seabed_interaction_solver = true;




    // Setting the touch_down_point position

//    Position tdp_position = GetPositionOnSeabed(m_Lb); // L'injecter dans la position du noeud TDP








  }

  double FrCatenaryLineSeabed::GetSeabedIntersection() const {
    auto seabed = GetSystem()->GetEnvironment()->GetOcean()->GetSeabed();

    // Intersection point searching using a bisection algorithm
    double sa = 0.;
    double sb = GetUnstretchedLength();
    while (std::fabs(sb - sa) > 1e-6) {

      double sm = 0.5 * (sa + sb);

      Position Pa = GetPositionInWorld(sa, NWU);
      double da = Pa.z() - seabed->GetBathymetry(Pa.x(), Pa.y(), NWU);

      Position Pm = GetPositionInWorld(sm, NWU);
      double dm = Pm.z() - seabed->GetBathymetry(Pm.x(), Pm.y(), NWU);

      if (da * dm <= 0.) {
        sb = sm;
      } else {
        sa = sm;
      }
    }
    return sa;
  }

  void FrCatenaryLineSeabed::solve() {

    if (!m_use_seabed_interaction_solver) {
      FrCatenaryLine::solve();
      return;
    }

    // TODO


  }

  Position FrCatenaryLineSeabed::GetPositionOnSeabed(double s) const {
    assert(0. <= s <= m_Lb);

    double gamma = m_Lb - m_t0.norm() / (c_Cb * m_q); // FIXME: est-ce bien cela qu'on doit prendre ???

    double lambda = (gamma > 0.) ? gamma : 0.;

    double lying_length = m_Lb +
                          (0.5 * c_Cb * m_q / m_properties->GetEA()) *
                          (m_Lb * m_Lb - 2. * m_Lb * gamma + gamma * lambda);



  }
}  // end namespace frydom
