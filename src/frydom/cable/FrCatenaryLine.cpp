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


#include "FrCatenaryLine.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/seabed/FrSeabed.h"

#include "frydom/asset/FrCatenaryLineAsset.h"

#include "frydom/logging/FrTypeNames.h"
#include "frydom/logging/FrEventLogger.h"

#include "boost/lexical_cast.hpp" // TODO: retirer, seulement pour tests
#include "boost/uuid/uuid_io.hpp"
#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_generators.hpp"


namespace frydom {


  FrCatenaryForce::FrCatenaryForce(const std::string &name, FrBody *body, FrCatenaryLine *line,
                                   FrCatenaryLine::LINE_SIDE side) :
      FrForce(name, TypeToString(this), body),
      m_line(line),
      m_line_side(side) {}

  bool FrCatenaryForce::IncludedInStaticAnalysis() const { return true; }

  void FrCatenaryForce::Compute(double time) {

    Position relative_position;
    Force force_in_world;

    // Get the line tension from the corresponding node
    switch (m_line_side) {
      case FrCatenaryLine::LINE_START:
        force_in_world = m_line->GetStartingNodeTension(NWU);
        relative_position = m_line->GetStartingNode()->GetNodePositionInBody(NWU);
        break;

      case FrCatenaryLine::LINE_END:
        force_in_world = m_line->GetEndingNodeTension(NWU);
        relative_position = m_line->GetEndingNode()->GetNodePositionInBody(NWU);
        break;
    }

    // Set the tension in the world reference frame and NWU frame convention
    SetForceTorqueInWorldAtPointInBody(force_in_world, Torque(), relative_position, NWU);

  }

  FrCatenaryLine::FrCatenaryLine(const std::string &name,
                                 const std::shared_ptr<FrNode> &startingNode,
                                 const std::shared_ptr<FrNode> &endingNode,
                                 const std::shared_ptr<FrCableProperties> &properties,
                                 bool elastic,
                                 double unstretchedLength,
                                 FLUID_TYPE fluid) :
      FrLoggable(name, TypeToString(this), startingNode->GetBody()->GetSystem()),
      FrPrePhysicsItem(),
      FrCable(startingNode, endingNode, properties, unstretchedLength),
      m_elastic(elastic),
      c_fluid(fluid),
      m_is_for_shape_initialization(false) {

    m_q = properties->GetLinearDensity();
  }

  FrCatenaryLine::FrCatenaryLine(const std::string &name,
                                 FrCable *cable,
                                 bool elastic,
                                 FLUID_TYPE fluid_type) :
      FrCatenaryLine(name,
                     cable->GetStartingNode(),
                     cable->GetEndingNode(),
                     cable->GetCableProperties(),
                     elastic,
                     cable->GetUnstretchedLength(),
                     fluid_type) {}

  void FrCatenaryLine::DefineLogMessages() {

    auto msg = NewMessage("State", "State messages");

    msg->AddField<double>("time", "s", "Current time of the simulation",
                          [this]() { return GetSystem()->GetTime(); });

    msg->AddField<double>("StrainedLength", "m", "Strained length of the catenary line",
                          [this]() { return GetStrainedLength(); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("StartingNodeTension", "N", fmt::format("Starting node tension in world reference frame in {}", GetLogFC()),
         [this]() { return GetStartingNodeTension(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("EndingNodeTension", "N", fmt::format("Ending node tension in world reference frame in {}", GetLogFC()),
         [this]() { return GetEndingNodeTension(GetLogFC()); });

    // TODO : logger la position de la ligne pour un ensemble d'abscisse curvilignes ?

  }

  void FrCatenaryLine::SetSolverTolerance(double tol) {
    m_tolerance = tol;
  }

  void FrCatenaryLine::SetSolverMaxIter(unsigned int maxiter) {
    m_itermax = maxiter;
  }

  void FrCatenaryLine::SetSolverInitialRelaxFactor(double relax) {
    m_relax = relax;
  }

  void FrCatenaryLine::guess_tension() {

    Position p0pL = GetEndingNode()->GetPositionInWorld(NWU) - GetStartingNode()->GetPositionInWorld(NWU);
    auto lx = p0pL[0];
    auto ly = p0pL[1];
    auto lz = p0pL[2];

    auto chord_length = p0pL.norm();
    auto v = m_u.cross(p0pL / chord_length).cross(m_u);

    double lambda = 0;
    if (m_unstretchedLength <= chord_length) {
      lambda = 0.2;
    } else if ((m_u.cross(p0pL)).norm() < 1e-4) {
      lambda = 1e6;
    } else {
      lambda = sqrt(3. * (m_unstretchedLength * m_unstretchedLength - lz * lz) / (lx * lx + ly * ly));
    }

    auto fu = -0.5 * m_q * (lz / tanh(lambda) - m_unstretchedLength);
    auto fv = 0.5 * m_q * sqrt(lx * lx + ly * ly) / lambda;

    m_t0 = fu * m_u + fv * v;
  }

  Force FrCatenaryLine::GetTension(double s, FRAME_CONVENTION fc) const {
    Force tension = m_t0 - c_qvec * s;
    if (IsNED(fc)) { internal::SwapFrameConvention(tension); }
    return tension;
  }

  void FrCatenaryLine::UseForShapeInitialization() {
    m_is_for_shape_initialization = true;
  }

  std::shared_ptr<FrCatenaryForce> FrCatenaryLine::GetStartingForce() {
    return m_startingForce;
  }

  std::shared_ptr<FrCatenaryForce> FrCatenaryLine::GetEndingForce() {
    return m_endingForce;
  }

  Force FrCatenaryLine::GetStartingNodeTension(FRAME_CONVENTION fc) const {
    return GetTension(0., fc);
  }

  Force FrCatenaryLine::GetEndingNodeTension(FRAME_CONVENTION fc) const {
    return -GetTension(GetUnstretchedLength(), fc);
  }

  Direction FrCatenaryLine::GetTangent(const double s, FRAME_CONVENTION fc) const {
    return (Direction) GetTension(s, fc).normalized();
  }

  void FrCatenaryLine::GetLowestPoint(Position &position,
                                      double &s,
                                      FRAME_CONVENTION fc,
                                      const double tol,
                                      const unsigned int maxIter) const {

    // Using a bisection algorithm to find the lowest point on the catenary line

    double s0 = 0.;
    double s1 = GetUnstretchedLength();

    auto p0 = GetStartingNode()->GetPositionInWorld(fc);
    auto p1 = GetEndingNode()->GetPositionInWorld(fc);

    double dz0 = GetTangent(s0, fc).z();
    double dz1 = GetTangent(s1, fc).z();

    // Dealing with border cases where the minimum point is at one of the boundary node
    if (dz0 * dz1 > 0.) {
      if (dz0 > 0.) {
        position = p0;
        s = s0;
      } else {
        position = p1;
        s = s1;
      }
      return;
    } else if (dz0 == 0.) {
      position = p0;
      s = s0;
      return;
    } else if (dz1 == 0.) {
      position = p1;
      s = s1;
      return;
    }

    // Bisection algorithm
    unsigned int iter = 0;
//        const unsigned int maxIter = 100;
    double dz = 0.;
    while (s1 - s0 > tol && iter < maxIter) {  // FIXME : dz ne change pas pendant les iterations !!!!!

      iter++;

      s = s0 + 0.5 * (s1 - s0);
      position = GetNodePositionInWorld(s, fc);

      dz = GetTangent(s, fc)[2];

      // See if derivative in z has changed sign
      if (dz0 * dz < 0) {
        s1 = s;
        p1 = position;
        dz1 = dz;
      } else if (dz1 * dz < 0) {
        s0 = s;
        p0 = position;
        dz0 = dz;
      } else break; // Tangent is horizontal, this is the minimum...

    }

    if (iter == maxIter) {
      event_logger::warn(GetTypeName(), GetName(),
                         "Maximum iteration in bisection has been reached while computing the lowest position in the element!");
    }
  }

  double FrCatenaryLine::_rho(double s) const {
    auto t0_qS = GetTension(s, NWU);
    return t0_qS.norm() - m_u.dot(t0_qS);
  }

  double FrCatenaryLine::GetUnstretchedLength() const {
    return m_unstretchedLength;
  }

  Position FrCatenaryLine::GetUnstretchedChord(double s, FRAME_CONVENTION fc) const {

    Position pc = -(m_u / m_q) * ((GetTension(s, NWU)).norm() - m_t0.norm());
    auto rho_0 = _rho(0.);  // TODO: calculer directement

    if (rho_0 > 0.) {
      auto rho_s = _rho(s);  // TODO: calculer directement
      if (rho_s > 0.) {
        pc += (c_Umat * m_t0 / m_q) * log(rho_s / rho_0);
      }
    }
    if (IsNED(fc)) { internal::SwapFrameConvention(pc); }
    return pc;

  }

  Position FrCatenaryLine::GetElasticIncrement(double s, FRAME_CONVENTION fc) const {
    Position Inc(0., 0., 0.);
    if (m_elastic) { Inc = m_q * s * (m_t0 / m_q - 0.5 * m_u * s) / m_properties->GetEA(); }
    if (IsNED(fc)) { internal::SwapFrameConvention(Inc); }
    return Inc;
  }

  Position FrCatenaryLine::GetNodePositionInWorld(double s, FRAME_CONVENTION fc) const {

    Position pos;
    pos += GetStartingNode()->GetPositionInWorld(fc);
    pos += GetUnstretchedChord(s, fc);
    pos += GetElasticIncrement(s, fc);
    return pos;

  }

  Position FrCatenaryLine::get_residual(FRAME_CONVENTION fc) const {
    return GetNodePositionInWorld(m_unstretchedLength, fc) - GetEndingNode()->GetPositionInWorld(fc);
  }

  bool FrCatenaryLine::HasSeabedInteraction() const {
    double s;
    Position lowest_position;
    GetLowestPoint(lowest_position, s, NWU, 1e-3, 20);

    return !(GetSystem()->GetEnvironment()->GetOcean()->GetSeabed()->IsAboveSeabed(lowest_position,
                                                                                   NWU)); // TODO: verifier !!
  }

  mathutils::Matrix33<double> FrCatenaryLine::analytical_jacobian() const {
    auto t0n = m_t0.norm();

    auto tL = m_t0 - c_qvec * m_unstretchedLength;
    auto tLn = tL.norm();

    auto rho_0 = _rho(0.);  // TODO: calculer directement
    double ln_q = 0.;
    double rho_L = 0.;
    if (rho_0 > 0.) {
      rho_L = _rho(m_unstretchedLength);  // TODO: calculer directement
      ln_q = log(rho_L / rho_0) / m_q;
    }

    double L_EA = 0.;
    if (m_elastic) L_EA = m_unstretchedLength / m_properties->GetEA();

    mathutils::Vector3d<double> Ui;
    double Uit0;
    double jac_ij;
    double diff_ln;
    auto jac = mathutils::Matrix33<double>();
    for (uint i = 0; i < 3; ++i) {
      Ui.x() = c_Umat(i, 0);
      Ui.y() = c_Umat(i, 1);
      Ui.z() = c_Umat(i, 2);

      Uit0 = Ui.dot(m_t0) / m_q;

      for (uint j = i; j < 3; ++j) {

        jac_ij = -(tL[j] / tLn - m_t0[j] / t0n) * m_u[i] / m_q;

        if (rho_0 > 0.) {
          jac_ij += c_Umat.at(i, j) * ln_q;
          diff_ln = (tL[j] / tLn - m_u[j]) / rho_L - (m_t0[j] / t0n - m_u[j]) / rho_0;
          jac_ij += Uit0 * diff_ln;
        }

        if (i == j) {  // elasticity
          jac_ij += L_EA;  // L_EA is null if no elasticity
          jac(i, j) = jac_ij;
        } else {
          jac(i, j) = jac_ij;
          jac(j, i) = jac_ij;
        }
      } // end for j
    } // end for i

    return jac;
  }

  void FrCatenaryLine::solve() {

    auto res = get_residual(NWU);
    auto jac = analytical_jacobian();

    jac.Inverse();
    mathutils::Vector3d<double> delta_t0 = jac * (-res);

    m_t0 += m_relax * delta_t0;

    res = get_residual(NWU);
    double err = res.infNorm();

    unsigned int iter = 1;
    while ((err > m_tolerance) && (iter < m_itermax)) {
      iter++;

      res = get_residual(NWU);
      jac = analytical_jacobian();

      jac.Inverse();
      mathutils::Vector3d<double> delta_t0_temp = jac * (-res);

      while (delta_t0.infNorm() < m_relax * delta_t0_temp.infNorm()) {
        m_relax *= 0.5;
        if (m_relax < Lmin) {
          event_logger::warn(GetTypeName(), GetName(), "DAMPING TOO STRONG. NO CATENARY CONVERGENCE.");
        }
      }

      delta_t0 = delta_t0_temp;
      m_t0 += m_relax * delta_t0;

      m_relax = std::min(1., m_relax * 2.);

      res = get_residual(NWU);
      err = res.infNorm();
    }  // end while

    if (iter == m_itermax) {
      event_logger::warn(GetTypeName(), GetName(), "Could not converge in max {} iterations", m_itermax);
    }

  }

  void FrCatenaryLine::Initialize() {

    m_q = m_properties->GetLinearDensity() -
          m_properties->GetSectionArea() * GetSystem()->GetEnvironment()->GetFluidDensity(c_fluid);
    m_q *= GetSystem()->GetGravityAcceleration();
    c_qvec = m_q * m_u;

    // Initializing U matrix
    c_Umat.SetIdentity();
    c_Umat -= m_u * (m_u.transpose().eval());

    // First guess for the tension
    // FIXME: supprimer ces initialize de node et mettre en place la séparation des SetupInitial des FrPhysicsItems en fonction des Pre, Mid et Post.
    m_startingNode->Initialize();
    m_endingNode->Initialize();
    guess_tension();
    solve();

    // C'est isi qu'on travaille pour faire de l'interaction seabed
    if (HasSeabedInteraction()) {
      std::cout << "SEABED INTERACTION" << std::endl;


      auto seabed = GetSystem()->GetEnvironment()->GetOcean()->GetSeabed();

      // Intersection point searching using a bisection algorithm
      double sa = 0.;
      double sb = GetUnstretchedLength();
      while (std::fabs(sb - sa) > 1e-6) {

        double sm = 0.5 * (sa + sb);

        Position Pa = GetNodePositionInWorld(sa, NWU);
        double da = Pa.z() - seabed->GetBathymetry(Pa.x(), Pa.y(), NWU);

        Position Pm = GetNodePositionInWorld(sm, NWU);
        double dm = Pm.z() - seabed->GetBathymetry(Pm.x(), Pm.y(), NWU);

        if (da * dm <= 0.) {
          sb = sm;
        } else {
          sa = sm;
        }

      }

//      Position tdp_position = GetNodePositionInWorld(sa, NWU);
      Direction dir = m_startingNode->GetPositionInWorld(NWU) - m_endingNode->GetPositionInWorld(NWU);
      dir.z() = 0.;
      dir /= dir.norm();

      double Ls = GetUnstretchedLength() - sa;
      Position tdp_position = m_endingNode->GetPositionInWorld(NWU) + Ls * dir;


      std::cout << "Intersection point: " << tdp_position << std::endl;


      // Distance ancre -> TDP
      // FIXME: il faut definir quelque chose pour declarer quel noeud est une ancre. En l'etat, ca ne fonctionnera pas
//      double Ls = (m_endingNode->GetPositionInWorld(NWU) - tdp_position).norm();

      std::cout << "TDP is at " << Ls << "meters from the anchor" << std::endl;

////      auto tdp_node = GetSystem()->GetWorldBody()->NewNode("tdp");
//      auto tdp_node = GetSystem()->GetEnvironment()->GetOcean()->GetSeabed()->NewAnchor(boost::lexical_cast<std::string>(boost::uuids::random_generator()()),
//          tdp_position.x(),
//                                                                                        tdp_position.y(), NWU);
//      tdp_node->SetPositionInWorld(tdp_position, NWU);
//
//      auto new_cable = make_catenary_line(boost::lexical_cast<std::string>(boost::uuids::random_generator()()),
//                                          m_startingNode,
//                                          tdp_node,
//                                          m_properties,
//                                          true,
//                                          sa,
//                                          c_fluid);
//      new_cable->Initialize();


    }


    if (!m_is_for_shape_initialization) {
      // Building the catenary forces and adding them to bodies
      if (!m_startingForce) {
        m_startingForce = std::make_shared<FrCatenaryForce>(GetName() + "_start_force", m_startingNode->GetBody(), this,
                                                            LINE_START);
        auto starting_body = m_startingNode->GetBody();
        starting_body->AddExternalForce(m_startingForce);
      }

      if (!m_endingForce) {
        m_endingForce = std::make_shared<FrCatenaryForce>(GetName() + "_end_force", m_endingNode->GetBody(), this,
                                                          LINE_END);
        auto ending_body = m_endingNode->GetBody();
        ending_body->AddExternalForce(m_endingForce);
      }

      FrCatenaryAssetOwner::Initialize();
    }

    FrCable::Initialize();
  }

  void FrCatenaryLine::Compute(double time) {

    UpdateTime(time);
    UpdateState();

  }

  void FrCatenaryLine::UpdateState() {
    FrCable::UpdateState();
    solve();
  }

  void FrCatenaryLine::StepFinalize() {
    FrAssetOwner::UpdateAsset();
    FrPhysicsItem::StepFinalize();
    FrObject::StepFinalize();
  }

  internal::FrPhysicsItemBase *FrCatenaryLine::GetChronoItem_ptr() const { return m_chronoPhysicsItem.get(); }


  std::shared_ptr<FrCatenaryLine>
  make_catenary_line(const std::string &name,
                     const std::shared_ptr<FrNode> &startingNode,
                     const std::shared_ptr<FrNode> &endingNode,
                     const std::shared_ptr<FrCableProperties> &properties,
                     bool elastic,
                     double unstretchedLength,
                     FLUID_TYPE fluid) {

    auto CatenaryLine = std::make_shared<FrCatenaryLine>(name,
                                                         startingNode,
                                                         endingNode,
                                                         properties,
                                                         elastic,
                                                         unstretchedLength,
                                                         fluid);
    startingNode->GetBody()->GetSystem()->Add(CatenaryLine);
    return CatenaryLine;

  }

}  // end namespace frydom
