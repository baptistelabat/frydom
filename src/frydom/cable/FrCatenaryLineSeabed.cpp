////
//// Created by frongere on 27/02/2020.
////
//
//#include "FrCatenaryLineSeabed.h"
//
//
//#include "frydom/logging/FrTypeNames.h"
//#include "frydom/logging/FrEventLogger.h"
//
//#include "frydom/core/common/FrNode.h"
//#include "frydom/core/body/FrBody.h"
//
//#include "frydom/environment/FrEnvironment.h"
//#include "frydom/environment/ocean/FrOcean.h"
//#include "frydom/environment/ocean/seabed/FrSeabed.h"
//
//namespace frydom {
//
//
//  FrCatenaryLineSeabed::FrCatenaryLineSeabed(const std::string &name,
//                                               const std::shared_ptr<FrNode> &anchorNode,
//                                               const std::shared_ptr<FrNode> &fairleadNode,
//                                               const std::shared_ptr<FrCableProperties> &properties,
//                                               bool elastic,
//                                               double unstretchedLength,
//                                               FLUID_TYPE fluid) :
//      FrCatenaryLine(name,
//                     anchorNode->GetBody()->NewNode(name + "_TDP_node"),
//                     fairleadNode,
//                     properties,
//                     elastic,
//                     unstretchedLength,
//                     fluid),
//      m_anchor_node(anchorNode),
//      m_Lb(0.),
//      c_Cb(1.),
//      m_use_seabed_interaction_solver(false) {
//
//  }
//
//  void FrCatenaryLineSeabed::Initialize() {
//
//    auto seabed = GetSystem()->GetEnvironment()->GetOcean()->GetSeabed();
//
//    // Testing if the seabed is not infinite
//    if (seabed->IsInfiniteDepth()) {
//      event_logger::error(GetTypeName(), GetName(),
//                          "Unable to build a seabed interaction line {} with infinite deabed depth", GetName());
//      exit(EXIT_FAILURE);
//    }
//
//    // We test that the anchor given is well on the seabed
//    if (!seabed->IsOnSeabed(m_anchor_node->GetPositionInWorld(NWU), NWU)) {
//      event_logger::error(GetTypeName(), GetName(), "Anchor node {} is not on seabed.", m_anchor_node->GetName());
//      exit(EXIT_FAILURE);
//    }
//
//
//
//    // Getting the cable seabed friction coeffient from seabed ??? pas de definition par ligne ???
//    c_Cb = 1.; // TODO: aller chercher le coeff dans FrSeabed !!! Constant par la suite donc on le met en cache
//
//    // Initially, we put the TDP at anchor position
//    GetTouchDownPointNode()->SetPositionInWorld(m_anchor_node->GetPositionInWorld(NWU), NWU);
//
//    FrCatenaryLine::Initialize();
//
//
//
//    // Getting the direction of the lying part of the line on seabed (intersection of the line plane and the seabed plane)
//    m_lying_direction = (GetFairleadNode()->GetPositionInWorld(NWU) -
//                         GetAnchorNode()->GetPositionInWorld(NWU)).normalized();
//    m_lying_direction = m_lying_direction.cross(m_u);
//    m_lying_direction = (m_lying_direction.cross(Direction(0, 0, 1))).normalized();
//
//
//    // Now we make a first guess on the TDP position by searching the intersection with the seabed
//    CorrectTouchDownPointAbscissae(GetSeabedIntersection() * 0.5);
//
//    // TDP node update
////    UpdateTouchDownPointPosition();
//    m_startingNode->SetPositionInWorld(m_anchor_node->GetPositionInWorld(NWU)
//                                       + m_Lb * m_lying_direction,
//                                       NWU);
////    Position estimated_TDP_position = m_anchor_node->GetPositionInWorld(NWU) + m_Lb * m_lying_direction;
//
//
////    Position estimated_TDP_position = GetPositionInWorld(m_Lb, NWU);
////
////
////    m_startingNode->SetPositionInWorld(estimated_TDP_position, NWU);
////    m_unstretchedLength -= m_Lb;
//
//    // Updating the catenary (to get also a first rough estimate of the tension at tdp)
//    guess_tension();
//    solve();
//
//    // Now we switch to the solver that deal with seabed interaction
//    m_use_seabed_interaction_solver = true;
//    solve();
//
//  }
//
//  void FrCatenaryLineSeabed::CorrectTouchDownPointAbscissae(const double &correction) {
//    m_Lb += correction;
//    m_unstretchedLength -= correction;
//  }
//
//  double FrCatenaryLineSeabed::GetSeabedIntersection() const {
//    auto seabed = GetSystem()->GetEnvironment()->GetOcean()->GetSeabed();
//
//
//    // Necessary for the bisection algorithm not to converge to the anchor position which is by definition on seabed
//    double L = GetUnstretchedLength();
//    double ds = L * 1e-3 / L;
//    double sa = ds;
//
//    Position position = GetPositionInWorld(sa, NWU);
//    while (seabed->IsOnSeabed(position, NWU) ||
//           seabed->IsAboveSeabed(position, NWU)) {
//      sa += ds;
//      if (sa > L) {
//        event_logger::error(GetTypeName(), GetName(),
//                            "This line {} is declared to have seabed interaction bu seems too taut to lie on seabed",
//                            GetName());
//        exit(EXIT_FAILURE);
//      }
//      position = GetPositionInWorld(sa, NWU);
//    }
//
//    // Intersection point searching using a bisection algorithm
//    double sb = GetUnstretchedLength();
//    while (std::fabs(sb - sa) > 1e-6) {
//
//      double sm = 0.5 * (sa + sb);
//
//      Position Pa = GetPositionInWorld(sa, NWU);
//      double da = Pa.z() - seabed->GetBathymetry(Pa.x(), Pa.y(), NWU);
//
//      Position Pm = GetPositionInWorld(sm, NWU);
//      double dm = Pm.z() - seabed->GetBathymetry(Pm.x(), Pm.y(), NWU);
//
//      if (da * dm <= 0.) {
//        sb = sm;
//      } else {
//        sa = sm;
//      }
//    }
//    return sa;
//  }
//
//  void FrCatenaryLineSeabed::solve() {
//
//    if (!m_use_seabed_interaction_solver) {
//      FrCatenaryLine::solve();
//      return;
//    }
//
//
//    mathutils::VectorN<double> res(4);
//    mathutils::MatrixMN<double> jac(4, 4);
//
//    res = get_residual_seabed();
//    std::cout << res << std::endl << std::endl;
//
//    jac = analytical_jacobian_seabed();
//
//    jac.Inverse();
//    mathutils::VectorN<double> correction(4);
//    correction = jac * (-res);
//
//
//    m_t0 += m_relax * correction.head(3);
//    CorrectTouchDownPointAbscissae(m_relax * correction.at(3));
//    m_startingNode->SetPositionInWorld(GetPositionInWorld(m_Lb, NWU), NWU);
////    FrCatenaryLine::solve();
//
//
//    res = get_residual_seabed();
//    std::cout << res << std::endl << std::endl;
//
//    double err = res.infNorm();
//
//    unsigned int iter = 1;
//
//    m_itermax = 10000; // FIXME: a retirer
//
//    while ((err > m_tolerance) && (iter < m_itermax)) {
//      iter++;
//
//      res = get_residual_seabed();
//      std::cout << res << std::endl << std::endl;
//
//      jac = analytical_jacobian_seabed();
//
//      jac.Inverse();
//      mathutils::VectorN<double> correction_temp(4);
//      correction_temp = jac * (-res);
//
//      while (correction.infNorm() < m_relax * correction_temp.infNorm()) {
//        m_relax *= 0.5;
//        if (m_relax < Lmin) {
//          event_logger::warn(GetTypeName(), GetName(), "DAMPING TOO STRONG. NO CATENARY CONVERGENCE.");
//        }
//      }
//
//      correction = correction_temp;
//      m_t0 += m_relax * correction.head(3);
//      CorrectTouchDownPointAbscissae(m_relax * correction.at(3));
//      m_startingNode->SetPositionInWorld(GetPositionInWorld(m_Lb, NWU), NWU);
////      FrCatenaryLine::solve();
//
//      m_relax = std::min(1., m_relax * 2.);
//
//      res = get_residual_seabed();
//      err = res.infNorm();
//    }  // end while
//
//    if (iter == m_itermax) {
//      event_logger::warn(GetTypeName(), GetName(), "Could not converge in max {} iterations", m_itermax);
//    }
//
//
//    std::cout << "cat line SEABED solver converged in " << iter << " iterations" << std::endl;
//
//
//  }
//
//  Position FrCatenaryLineSeabed::GetPositionOnSeabed(double s, FRAME_CONVENTION fc) const {
//    assert(0. <= s && s <= m_Lb);
//
//    Position position;
//
//    double H = GetHorizontalTensionAtTouchDownPoint();
//
//    double gamma = m_Lb - H /
//                          (c_Cb * m_q); // TODO: voir a mettre en cache ! c'est fixe suivant la configuration !!!
//
//    if (s <= gamma) {
//      // From anchor (s=0) to gamma, there is no tension
//      Position offset = m_lying_direction * s;
//      if (IsNED(fc)) { internal::SwapFrameConvention(offset); }
//      position = m_anchor_node->GetPositionInWorld(fc) + offset;
//
//    } else {
//      double lambda = (gamma > 0.) ? gamma : 0.;
//      Position offset = (s + (0.5 * c_Cb * m_q / m_properties->GetEA()) * (s * s - 2. * s * gamma + gamma * lambda)) *
//                        m_lying_direction;
//
//      position = m_anchor_node->GetPositionInWorld(fc) + offset;
//
//    }
//
//    return position;
//  }
//
//  Position FrCatenaryLineSeabed::GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const {
//    assert(0. <= s);
//
//    if (!m_use_seabed_interaction_solver)
//      return FrCatenaryLine::GetPositionInWorld(s, fc);
//
//    if (s <= m_Lb) {
//      return GetPositionOnSeabed(s, fc);
//    } else {
//      return FrCatenaryLine::GetPositionInWorld(s - m_Lb, fc);
//    }
//
//  }
//
//  Force FrCatenaryLineSeabed::GetTension(const double &s, FRAME_CONVENTION fc) const {
//    assert(0. <= s && s <= GetUnstretchedLength());
//
//    Force tension;
//
//    if (!m_use_seabed_interaction_solver) {
//      return FrCatenaryLine::GetTension(s, fc);
//    }
//
//    if (s <= m_Lb) {
//      double tx = m_t0.GetFx();
//      double ty = m_t0.GetFy();
//      double H = GetHorizontalTensionAtTouchDownPoint();
//      tension = std::max(H + c_Cb * m_q * (s - m_Lb), 0.) * m_lying_direction;
//      if (IsNED(fc))
//        internal::SwapFrameConvention(tension);
//      // TODO: verifier qu'on est bien nul si plus petit que gamma...
//
//    } else {
//      tension = FrCatenaryLine::GetTension(s - m_Lb, fc);
//    }
//
//    return tension;
//  }
//
//  inline double FrCatenaryLineSeabed::GetHorizontalTensionAtTouchDownPoint() const {
//    double tx = m_t0.GetFx();
//    double ty = m_t0.GetFy();
//    return std::sqrt(tx * tx + ty * ty);
//  }
//
//  double FrCatenaryLineSeabed::GetUnstretchedLengthCatenaryPart() const {
//    return m_unstretchedLength;
//  }
//
//  double FrCatenaryLineSeabed::GetUnstretchedLength() const {
//    return m_unstretchedLength + m_Lb;
//  }
//
//  mathutils::VectorN<double> FrCatenaryLineSeabed::get_residual_seabed() const {
//    mathutils::VectorN<double> residual(4);
//    residual.head(3) = get_residual(NWU);
//    residual.at(3) = m_q * (m_unstretchedLength) + m_u.transpose() * (m_t0 - m_q * m_unstretchedLength * m_u);
//    return residual;
//  }
//
//  mathutils::MatrixMN<double> FrCatenaryLineSeabed::analytical_jacobian_seabed() const {
//
//    mathutils::MatrixMN<double> jacobian(4, 4);
//
//    jacobian.topLeftCorner(3, 3) = analytical_jacobian();
//
//    // Position part with respect to Lb
//    auto alpha = m_u.transpose() * GetTension(GetUnstretchedLength(), NWU).normalized();
//
//    jacobian.topRightCorner(3, 1) =
//        (c_Umat * m_t0 / _rho(m_unstretchedLength)) * (alpha - m_u.transpose() * m_u) - alpha * m_u - m_t0 / m_q;
//    // TODO: voir si c'est reellement GetUnstretchedLength() ou bien la longugueur dans l'eau...
//
//
//    jacobian.bottomLeftCorner(1, 3) = m_u.transpose();
//
//    jacobian.at(3, 3) = m_q * (1 - m_u.transpose() * m_u);
//
//    return jacobian;
//
//  }
//
//
//}  // end namespace frydom
