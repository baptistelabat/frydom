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

#include <Eigen/Dense>

#include "FrCatenaryLine.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/seabed/FrSeabed.h"

#include "frydom/logging/FrTypeNames.h"
#include "frydom/logging/FrEventLogger.h"

namespace frydom {

  FrCatenaryLine::FrCatenaryLine(const std::string &name,
                                 const std::shared_ptr<FrNode> &startingNode,
                                 const std::shared_ptr<FrNode> &endingNode,
                                 const std::shared_ptr<FrCableProperties> &properties,
                                 bool elastic,
                                 double unstretchedLength,
                                 FLUID_TYPE fluid_type) :
      FrCatenaryLineBase(name, TypeToString(this), startingNode, endingNode, properties, elastic, unstretchedLength),
      c_qL(0.) {

    m_point_forces.emplace_back(internal::PointForce{0., Force()});
    m_q = properties->GetLinearDensity() - properties->GetSectionArea() *
                                           startingNode->GetSystem()->GetEnvironment()->GetFluidDensity(fluid_type);
    m_pi = {0., 0., -1.};
  }

  FrCatenaryLine::FrCatenaryLine(const std::string &name, FrCable *cable, bool elastic, FLUID_TYPE fluid_type) :
      FrCatenaryLine(name,
                     cable->GetStartingNode(),
                     cable->GetEndingNode(),
                     cable->GetCableProperties(),
                     elastic,
                     cable->GetUnstretchedLength(),
                     fluid_type) {}

  void FrCatenaryLine::AddClumpWeight(const double &s, const double &mass) {
    AddPointMass(s / m_unstretchedLength,
                 Force(0, 0, -mass * GetSystem()->GetEnvironment()->GetGravityAcceleration()) / c_qL);
    // FIXME: il faut que c_qL soit set dans le constructeur !
  }

  void FrCatenaryLine::AddBuoy(const double &s, const double &mass) {
    AddPointMass(s / m_unstretchedLength,
                 Force(0, 0, mass * GetSystem()->GetEnvironment()->GetGravityAcceleration()) / c_qL);
  }

  bool FrCatenaryLine::IsSingular() const {
    return rho(0, 0.) == 0. || rho(N(), 1.) == 0.; // TODO: voir dans catway pour les tests...
  }

  void FrCatenaryLine::Initialize() {

    m_startingNode->Initialize();
    m_endingNode->Initialize();

    BuildCache();

    GuessTension();
    solve();

    if (!m_use_for_shape_initialization) {
      // Building the catenary forces and adding them to bodies
      if (!m_startingForce) {
        m_startingForce = std::make_shared<FrCatenaryForce>(GetName() + "_start_force", m_startingNode->GetBody(), this,
                                                            FrCatenaryLineBase::LINE_START);
        auto starting_body = m_startingNode->GetBody();
        starting_body->AddExternalForce(m_startingForce);
      }

      if (!m_endingForce) {
        m_endingForce = std::make_shared<FrCatenaryForce>(GetName() + "_end_force", m_endingNode->GetBody(), this,
                                                          FrCatenaryLineBase::LINE_END);
        auto ending_body = m_endingNode->GetBody();
        ending_body->AddExternalForce(m_endingForce);
      }

      FrCatenaryAssetOwner::Initialize();
    }

    FrCable::Initialize();

  }

  Force FrCatenaryLine::GetTension(const double &s, FRAME_CONVENTION fc) const {
    Tension tension = t(s) * c_qL;
    if (IsNED(fc))
      internal::SwapFrameConvention(tension);
    return tension;
  }

  Position FrCatenaryLine::GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const {
    Position position = p(s) * m_unstretchedLength;
    if (IsNED(fc))
      internal::SwapFrameConvention(position);
    return position;
  }

  double FrCatenaryLine::GetUnstretchedLength() const {
    return m_unstretchedLength;
  }

//  bool FrCatenaryLine::HasSeabedInteraction() const {
//    // TODO
//  }

  void FrCatenaryLine::solve() {

    unsigned int iter = 0;

    // Defining the linear solver
    Eigen::FullPivLU<Eigen::Matrix3d> linear_solver;

    Residue3 residue = GetResidue();
    double err = residue.infNorm();
    if (err < m_tolerance) {
      std::cout << "Already at equilibrium" << std::endl;
      return;
    }

    while (iter < m_maxiter) {

      iter++;

      Jacobian33 jacobian = GetJacobian();

      linear_solver.compute(jacobian);
      Tension dt0 = linear_solver.solve(-residue);

      m_t0 += dt0;

      residue = GetResidue();
      err = residue.infNorm();

      if (err < m_tolerance) {
        std::cout << "convergence in " << iter << std::endl;
        break;
      }

    }

    if (iter == m_maxiter) {
      std::cout << "NO CONVERGENCE" << std::endl;
    } else {
      std::cout << "CONVERGENCE IN " << iter << std::endl;
    }

  }

  void FrCatenaryLine::AddPointMass(const double &s, const Force &force) {
    // TODO:gerer le fait qu'on decrit s a partir du fairlead ???
    auto pos = m_point_forces.cbegin();
    for (; pos != m_point_forces.cend(); pos++) {
      if (s > pos->s())
        break;
    }
    m_point_forces.insert(pos, internal::PointForce(s, force));
  }

  auto FrCatenaryLine::ti(const unsigned int &i, const double &s) const {
    return m_t0 - Fi(i) - m_pi * s;
  }

  double FrCatenaryLine::rho(const unsigned int &i, const double &s) const {
    return ti(i, s).norm() - m_pi.transpose() * ti(i, s); // On essaie d'exploiter Eigen...
  }

  double FrCatenaryLine::lambda(const unsigned int &i, const double &s) const {
    return std::log(rho(i, s));
  }

  Force FrCatenaryLine::Fi(const unsigned int &i) const {
    return c_Fi[i];
  }

  Force FrCatenaryLine::fi(const unsigned int &i) const {
    return m_point_forces[i].force();
  }

  double FrCatenaryLine::si(const unsigned int &i) const {
    return m_point_forces[i].s();
  }

  auto FrCatenaryLine::Lambda_tau(const unsigned int &i, const double &s) const {
    double ti_s_n = ti(i, s).norm();
    return (m_t0 - ti_s_n * m_pi) / (rho(i, s) * ti_s_n);
  }

  unsigned int FrCatenaryLine::N() const { return m_point_forces.size() - 1; }

  FrCatenaryLineBase::Tension FrCatenaryLine::t(const double &s) const {
    unsigned int i = SToI(s);
    return ti(i, s);
  }

  FrCatenaryLineBase::Tension FrCatenaryLine::tL() const {
    return t(1.);
  }

  Position FrCatenaryLine::p0() const {
    return m_startingNode->GetPositionInWorld(NWU) / m_unstretchedLength;
  }

  unsigned int FrCatenaryLine::SToI(const double &s) const {
    assert(0. <= s && s <= 1.);

    unsigned int i = 0;
    // si < s <= si+1 ---> we send back i
    for (unsigned int j = 0; i < N(); i++) {
      if (si(j) < s && s <= si(j + 1)) {
        i = j;
        break;
      }
    }
    return i;
  }

  Position FrCatenaryLine::p_pi(const unsigned int &i, const double &s) const {
    auto scalar = ti(i, s).norm() - ti(i, si(i)).norm();
    for (int j = 0; j < i; j++) {
      scalar += ti(j, si(j + 1)).norm() - ti(j, si(j)).norm();
    }
    return -m_pi * scalar;
  }

  Position FrCatenaryLine::p_perp(const unsigned int &i, const double &s) const {
    Position vector = (m_t0 - Fi(i)) * (lambda(i, s) - lambda(i, si(i)));
    for (int j = 0; j < i; j++) {
      vector += (m_t0 - Fi(j)) * (lambda(j, si(j + 1)) - lambda(j, si(j)));
    }
    return c_U * vector;
  }

  Position FrCatenaryLine::pc(const unsigned int &i, const double &s) const {
    if (IsSingular()) {
      return p_pi(i, s);
    } else {
      return p_pi(i, s) + p_perp(i, s);
    }
  }

  Force FrCatenaryLine::sum_fs(const unsigned int &i) const {
    return c_sum_fs[i];
  }

  Position FrCatenaryLine::pe(const unsigned int &i, const double &s) const {
    return (m_q * m_unstretchedLength / m_properties->GetEA()) *
           ((m_t0 - Fi(i)) * s + sum_fs(i) - 0.5 * m_pi * s * s);
  }

  Position FrCatenaryLine::pi(const unsigned int &i, const double &s) const {
    Position p = p0() + pc(i, s);
    if (m_elastic)
      p += pe(i, s);
    return p;
  }

  Position FrCatenaryLine::p(const double &s) const {
    unsigned int i = SToI(s);
    return pi(i, s);
  }

  Position FrCatenaryLine::pL() const {
    return m_endingNode->GetPositionInWorld(NWU) / m_unstretchedLength;
  }

  void FrCatenaryLine::GuessTension() {
    // Peyrot et goulois

    Position p0pL = GetEndingNode()->GetPositionInWorld(NWU) - GetStartingNode()->GetPositionInWorld(NWU);
    double lx = p0pL[0];
    double ly = p0pL[1];
    double lz = p0pL[2];

    double chord_length = p0pL.norm();
    Direction v = m_pi.cross(p0pL / chord_length).cross(m_pi);

    double lambda = 0;
    if (m_unstretchedLength <= chord_length) {
      lambda = 0.2;
    } else if ((m_pi.cross(p0pL)).norm() < 1e-4) {
      lambda = 1e6;
    } else {
      lambda = sqrt(3. * (m_unstretchedLength * m_unstretchedLength - lz * lz) / (lx * lx + ly * ly));
    }

    auto fu = -0.5 * m_q * (lz / std::tanh(lambda) - m_unstretchedLength);
    auto fv = 0.5 * m_q * std::sqrt(lx * lx + ly * ly) / lambda;

    m_t0 = (fu * m_pi + fv * v) / c_qL;
  }

  FrCatenaryLine::Residue3 FrCatenaryLine::GetResidue() const {
    auto pend = pL();
    return p(1.) - pL(); // TODO : verifier
  }

  FrCatenaryLine::Jacobian33 FrCatenaryLine::dp_pi_dt() const {
    double scalar = 1. / ti(N(), 1.).norm() - 1. / ti(N(), si(N())).norm();
    for (unsigned int j = 0; j < N(); j++) { // FIXME: verifier le N-1 !!!!!!!!!!!! ---> BUG
      scalar += 1. / ti(j, si(j + 1)).norm() - 1. / ti(j, si(j)).norm();
    }
    return -scalar * m_pi * m_t0.transpose();
  }

  FrCatenaryLine::Jacobian33 FrCatenaryLine::dp_perp_dt() const {
    Jacobian33 matrix = (m_t0 - Fi(N())) * (Lambda_tau(N(), 1.) - Lambda_tau(N(), si(N()))).transpose();
    double scalar = lambda(N(), 1.) - lambda(N(), si(N()));
    for (unsigned int j = 0; j < N(); j++) {
      matrix += (m_t0 - Fi(j)) * (Lambda_tau(j, si(j + 1)) - Lambda_tau(j, si(j))).transpose();
      scalar += lambda(j, si(j + 1)) - lambda(j, si(j));
    }
    return c_U * matrix + scalar * Eigen::Matrix3d::Identity();
  }

  FrCatenaryLine::Jacobian33 FrCatenaryLine::dpc_dt() const {
    auto jacobian = dp_pi_dt();
    if (!IsSingular()) {
      jacobian += dp_perp_dt();
    }
    return jacobian;
  }

  FrCatenaryLine::Jacobian33 FrCatenaryLine::dpe_dt() const {
    return (m_q * m_unstretchedLength / m_properties->GetEA()) * Eigen::Matrix3d::Identity(); // TODO: optim
  }

  FrCatenaryLine::Jacobian33 FrCatenaryLine::GetJacobian() const {
//    auto jacobian;
    auto jacobian = dpc_dt();
    if (m_elastic) {
      jacobian += dpe_dt();
    }
    return jacobian;
  }

  void FrCatenaryLine::Compute(double time) {
    // TODO
  }

  internal::FrPhysicsItemBase *FrCatenaryLine::GetChronoItem_ptr() const {
    // TODO
  }

  void FrCatenaryLine::DefineLogMessages() {
    // TODO
  }

  void FrCatenaryLine::BuildCache() {
    c_U.SetIdentity();
    c_U -= m_pi * m_pi.transpose();

    c_qL = m_q * m_unstretchedLength;

    c_Fi.clear();
    c_sum_fs.clear();
    c_Fi.emplace_back(m_point_forces.front().force());
    c_sum_fs.emplace_back(m_point_forces.front().force());

    for (unsigned int i = 0; i < N(); i++) { // TODO: voir la borne
      auto point_force = m_point_forces[i + 1];
      c_Fi.emplace_back(c_Fi[i] + point_force.force());
      c_sum_fs.emplace_back(c_sum_fs[i] + point_force.force() * point_force.s());
    }
  }





























//  FrCatenaryForce::FrCatenaryForce(const std::string &name,
//                                   FrBody *body,
//                                   FrCatenaryLine *line,
//                                   FrCatenaryLine::LINE_SIDE side) :
//      FrForce(name, TypeToString(this), body),
//      m_line(line),
//      m_line_side(side) {}
//
//  bool FrCatenaryForce::IncludedInStaticAnalysis() const { return true; }
//
//  void FrCatenaryForce::Compute(double time) {
//
//    Position relative_position;
//    Force force_in_world;
//
//    // Get the line tension from the corresponding node
//    switch (m_line_side) {
//      case FrCatenaryLine::LINE_START:
//        force_in_world = m_line->GetStartingNodeTension(NWU);
//        relative_position = m_line->GetStartingNode()->GetNodePositionInBody(NWU);
//        break;
//
//      case FrCatenaryLine::LINE_END:
//        force_in_world = m_line->GetEndingNodeTension(NWU);
//        relative_position = m_line->GetEndingNode()->GetNodePositionInBody(NWU);
//        break;
//    }
//
//    // Set the tension in the world reference frame and NWU frame convention
//    SetForceTorqueInWorldAtPointInBody(force_in_world, Torque(), relative_position, NWU);
//
//  }




//
//  FrCatenaryLine_::FrCatenaryLine_(const std::string &name,
//                                   const std::shared_ptr<FrNode> &startingNode,
//                                   const std::shared_ptr<FrNode> &endingNode,
//                                   const std::shared_ptr<FrCableProperties> &properties,
//                                   bool elastic,
//                                   double unstretchedLength,
//                                   FLUID_TYPE fluid) :
//      FrLoggable(name, TypeToString(this), startingNode->GetBody()->GetSystem()),
//      FrPrePhysicsItem(),
//      FrCable(startingNode, endingNode, properties, unstretchedLength),
//      m_elastic(elastic),
//      c_fluid(fluid),
//      m_is_for_shape_initialization(false) {
//
//    m_q = properties->GetLinearDensity();
//  }
//
//  FrCatenaryLine_::FrCatenaryLine_(const std::string &name,
//                                   FrCable *cable,
//                                   bool elastic,
//                                   FLUID_TYPE fluid_type) :
//      FrCatenaryLine_(name,
//                      cable->GetStartingNode(),
//                      cable->GetEndingNode(),
//                      cable->GetCableProperties(),
//                      elastic,
//                      cable->GetUnstretchedLength(),
//                      fluid_type) {}
//
//  void FrCatenaryLine_::DefineLogMessages() {
//
//    auto msg = NewMessage("State", "State messages");
//
//    msg->AddField<double>("time", "s", "Current time of the simulation",
//                          [this]() { return GetSystem()->GetTime(); });
//
//    msg->AddField<double>("StrainedLength", "m", "Strained length of the catenary line",
//                          [this]() { return GetStrainedLength(); });
//
//    msg->AddField<Eigen::Matrix<double, 3, 1>>
//        ("StartingNodeTension", "N", fmt::format("Starting node tension in world reference frame in {}", GetLogFC()),
//         [this]() { return GetStartingNodeTension(GetLogFC()); });
//
//    msg->AddField<Eigen::Matrix<double, 3, 1>>
//        ("EndingNodeTension", "N", fmt::format("Ending node tension in world reference frame in {}", GetLogFC()),
//         [this]() { return GetEndingNodeTension(GetLogFC()); });
//
//    // TODO : logger la position de la ligne pour un ensemble d'abscisse curvilignes ?
//
//  }
//
//  void FrCatenaryLine_::SetSolverTolerance(double tol) {
//    m_tolerance = tol;
//  }
//
//  void FrCatenaryLine_::SetSolverMaxIter(unsigned int maxiter) {
//    m_itermax = maxiter;
//  }
//
//  void FrCatenaryLine_::SetSolverInitialRelaxFactor(double relax) {
//    m_relax = relax;
//  }
//
//  void FrCatenaryLine_::guess_tension() {
//
//    Position p0pL = GetEndingNode()->GetPositionInWorld(NWU) - GetStartingNode()->GetPositionInWorld(NWU);
//    auto lx = p0pL[0];
//    auto ly = p0pL[1];
//    auto lz = p0pL[2];
//
//    auto chord_length = p0pL.norm();
//    auto v = m_u.cross(p0pL / chord_length).cross(m_u);
//
//    double lambda = 0;
//    if (m_unstretchedLength <= chord_length) {
//      lambda = 0.2;
//    } else if ((m_u.cross(p0pL)).norm() < 1e-4) {
//      lambda = 1e6;
//    } else {
//      lambda = sqrt(3. * (m_unstretchedLength * m_unstretchedLength - lz * lz) / (lx * lx + ly * ly));
//    }
//
//    auto fu = -0.5 * m_q * (lz / tanh(lambda) - m_unstretchedLength);
//    auto fv = 0.5 * m_q * sqrt(lx * lx + ly * ly) / lambda;
//
//    m_t0 = fu * m_u + fv * v;
//  }
//
//  Force FrCatenaryLine_::GetTension(const double &s, FRAME_CONVENTION fc) const {
//    Force tension = m_t0 - c_qvec * s;
//    if (IsNED(fc)) { internal::SwapFrameConvention(tension); }
//    return tension;
//  }
//
//  void FrCatenaryLine_::UseForShapeInitialization() {
//    m_is_for_shape_initialization = true;
//  }
//
//  std::shared_ptr<FrCatenaryForce> FrCatenaryLine_::GetStartingForce() {
//    return m_startingForce;
//  }
//
//  std::shared_ptr<FrCatenaryForce> FrCatenaryLine_::GetEndingForce() {
//    return m_endingForce;
//  }
//
//  Force FrCatenaryLine_::GetStartingNodeTension(FRAME_CONVENTION fc) const {
//    return FrCatenaryLine_::GetTension(0., fc);
//  }
//
//  Force FrCatenaryLine_::GetEndingNodeTension(FRAME_CONVENTION fc) const {
//    return -FrCatenaryLine_::GetTension(FrCatenaryLine_::GetUnstretchedLength(), fc);
//  }
//
//  Direction FrCatenaryLine_::GetTangent(const double s, FRAME_CONVENTION fc) const {
//    return (Direction) FrCatenaryLine_::GetTension(s, fc).normalized();
//  }
//
//  void FrCatenaryLine_::GetLowestPoint(Position &position,
//                                       double &s,
//                                       FRAME_CONVENTION fc,
//                                       const double tol,
//                                       const unsigned int maxIter) const {
//
//    // Using a bisection algorithm to find the lowest point on the catenary line
//
//    double s0 = 0.;
//    double s1 = FrCatenaryLine_::GetUnstretchedLength();
//
//    auto p0 = GetStartingNode()->GetPositionInWorld(fc);
//    auto p1 = GetEndingNode()->GetPositionInWorld(fc);
//
//    double dz0 = FrCatenaryLine_::GetTangent(s0, fc).z();
//    double dz1 = FrCatenaryLine_::GetTangent(s1, fc).z();
//
//    // Dealing with border cases where the minimum point is at one of the boundary node
//    if (dz0 * dz1 > 0.) {
//      if (dz0 > 0.) {
//        position = p0;
//        s = s0;
//      } else {
//        position = p1;
//        s = s1;
//      }
//      return;
//    } else if (dz0 == 0.) {
//      position = p0;
//      s = s0;
//      return;
//    } else if (dz1 == 0.) {
//      position = p1;
//      s = s1;
//      return;
//    }
//
//    // Bisection algorithm
//    unsigned int iter = 0;
////        const unsigned int maxIter = 100;
//    double dz = 0.;
//    while (s1 - s0 > tol && iter < maxIter) {  // FIXME : dz ne change pas pendant les iterations !!!!!
//
//      iter++;
//
//      s = s0 + 0.5 * (s1 - s0);
//      position = FrCatenaryLine_::GetPositionInWorld(s, fc);
//
//      dz = GetTangent(s, fc)[2];
//
//      // See if derivative in z has changed sign
//      if (dz0 * dz < 0) {
//        s1 = s;
//        p1 = position;
//        dz1 = dz;
//      } else if (dz1 * dz < 0) {
//        s0 = s;
//        p0 = position;
//        dz0 = dz;
//      } else break; // Tangent is horizontal, this is the minimum...
//
//    }
//
//    if (iter == maxIter) {
//      event_logger::warn(GetTypeName(), GetName(),
//                         "Maximum iteration in bisection has been reached while computing the lowest position in the element!");
//    }
//  }
//
//  double FrCatenaryLine_::_rho(double s) const {
//    auto t0_qS = FrCatenaryLine_::GetTension(s, NWU);
//    return t0_qS.norm() - m_u.dot(t0_qS);
//  }
//
//  double FrCatenaryLine_::GetUnstretchedLength() const {
//    return m_unstretchedLength;
//  }
//
//  Position FrCatenaryLine_::GetUnstretchedChord(double s, FRAME_CONVENTION fc) const {
//
//    Position pc = -(m_u / m_q) * ((FrCatenaryLine_::GetTension(s, NWU)).norm() - m_t0.norm());
//    auto rho_0 = _rho(0.);  // TODO: calculer directement
//
//    if (rho_0 > 0.) {
//      auto rho_s = _rho(s);  // TODO: calculer directement
//      if (rho_s > 0.) {
//        pc += (c_Umat * m_t0 / m_q) * log(rho_s / rho_0);
//      }
//    }
//    if (IsNED(fc)) { internal::SwapFrameConvention(pc); }
//    return pc;
//
//  }
//
//  Position FrCatenaryLine_::GetElasticIncrement(double s, FRAME_CONVENTION fc) const {
//    Position Inc(0., 0., 0.);
//    if (m_elastic) { Inc = m_q * s * (m_t0 / m_q - 0.5 * m_u * s) / m_properties->GetEA(); }
//    if (IsNED(fc)) { internal::SwapFrameConvention(Inc); }
//    return Inc;
//  }
//
//  Position FrCatenaryLine_::GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const {
//    assert(0. <= s <= m_unstretchedLength);
//    Position pos;
//    pos += GetStartingNode()->GetPositionInWorld(fc);
//    pos += GetUnstretchedChord(s, fc);
//    pos += GetElasticIncrement(s, fc);
//    return pos;
//
//  }
//
//  Position FrCatenaryLine_::get_residual(FRAME_CONVENTION fc) const {
//    return FrCatenaryLine_::GetPositionInWorld(FrCatenaryLine_::GetUnstretchedLength(), fc) -
//           GetEndingNode()->GetPositionInWorld(fc);
//  }
//
//  bool FrCatenaryLine_::HasSeabedInteraction() const {
//    double s;
//    Position lowest_position;
//    GetLowestPoint(lowest_position, s, NWU, 1e-3, 20);
//
//    return !(GetSystem()->GetEnvironment()->GetOcean()->GetSeabed()->IsAboveSeabed(lowest_position,
//                                                                                   NWU)); // TODO: verifier !!
//  }
//
//  mathutils::Matrix33<double> FrCatenaryLine_::analytical_jacobian() const {
//    auto t0n = m_t0.norm();
//
//    auto tL = m_t0 - c_qvec * m_unstretchedLength;
//    auto tLn = tL.norm();
//
//    auto rho_0 = _rho(0.);  // TODO: calculer directement
//    double ln_q = 0.;
//    double rho_L = 0.;
//    if (rho_0 > 0.) {
//      rho_L = _rho(m_unstretchedLength);  // TODO: calculer directement
//      ln_q = log(rho_L / rho_0) / m_q;
//    }
//
//    double L_EA = 0.;
//    if (m_elastic) L_EA = m_unstretchedLength / m_properties->GetEA();
//
//    mathutils::Vector3d<double> Ui;
//    double Uit0;
//    double jac_ij;
//    double diff_ln;
//    auto jac = mathutils::Matrix33<double>();
//    for (uint i = 0; i < 3; ++i) {
//      Ui.x() = c_Umat(i, 0);
//      Ui.y() = c_Umat(i, 1);
//      Ui.z() = c_Umat(i, 2);
//
//      Uit0 = Ui.dot(m_t0) / m_q;
//
//      for (uint j = i; j < 3; ++j) {
//
//        jac_ij = -(tL[j] / tLn - m_t0[j] / t0n) * m_u[i] / m_q;
//
//        if (rho_0 > 0.) {
//          jac_ij += c_Umat.at(i, j) * ln_q;
//          diff_ln = (tL[j] / tLn - m_u[j]) / rho_L - (m_t0[j] / t0n - m_u[j]) / rho_0;
//          jac_ij += Uit0 * diff_ln;
//        }
//
//        if (i == j) {  // elasticity
//          jac_ij += L_EA;  // L_EA is null if no elasticity
//          jac(i, j) = jac_ij;
//        } else {
//          jac(i, j) = jac_ij;
//          jac(j, i) = jac_ij;
//        }
//      } // end for j
//    } // end for i
//
//    return jac;
//  }
//
//  void FrCatenaryLine_::solve() {
//
//    auto res = get_residual(NWU);
//    auto jac = analytical_jacobian();
//
//    jac.Inverse();
//    mathutils::Vector3d<double> delta_t0 = jac * (-res);
//
//    m_t0 += m_relax * delta_t0;
//
//    res = get_residual(NWU);
//    double err = res.infNorm();
//
//    unsigned int iter = 1;
//    while ((err > m_tolerance) && (iter < m_itermax)) {
//      iter++;
//
//      res = get_residual(NWU);
//      jac = analytical_jacobian();
//
//      jac.Inverse();
//      mathutils::Vector3d<double> delta_t0_temp = jac * (-res);
//
//      while (delta_t0.infNorm() < m_relax * delta_t0_temp.infNorm()) {
//        m_relax *= 0.5;
//        if (m_relax < Lmin) {
//          event_logger::warn(GetTypeName(), GetName(), "DAMPING TOO STRONG. NO CATENARY CONVERGENCE.");
//        }
//      }
//
//      delta_t0 = delta_t0_temp;
//      m_t0 += m_relax * delta_t0;
//
//      m_relax = std::min(1., m_relax * 2.);
//
//      res = get_residual(NWU);
//      err = res.infNorm();
//    }  // end while
//
//    if (iter == m_itermax) {
//      event_logger::warn(GetTypeName(), GetName(), "Could not converge in max {} iterations", m_itermax);
//    }
//
//    std::cout << "cat line solver converged in " << iter << " iterations" << std::endl;
//
//  }
//
//  void FrCatenaryLine_::Initialize() {
//
//    m_q = m_properties->GetLinearDensity() -
//          m_properties->GetSectionArea() * GetSystem()->GetEnvironment()->GetFluidDensity(c_fluid);
//    m_q *= GetSystem()->GetGravityAcceleration();
//    c_qvec = m_q * m_u;
//
//    // Initializing U matrix
//    c_Umat.SetIdentity();
//    c_Umat -= m_u * (m_u.transpose().eval()); // FIXME: pourquoi le eval ??
//
//    // First guess for the tension
//    // FIXME: supprimer ces initialize de node et mettre en place la séparation des SetupInitial des FrPhysicsItems en fonction des Pre, Mid et Post.
//    m_startingNode->Initialize();
//    m_endingNode->Initialize();
//    guess_tension();
//    FrCatenaryLine_::solve();
//
//    if (!m_is_for_shape_initialization) {
//      // Building the catenary forces and adding them to bodies
//      if (!m_startingForce) {
//        m_startingForce = std::make_shared<FrCatenaryForce>(GetName() + "_start_force", m_startingNode->GetBody(), this,
//                                                            FrCatenaryLineBase::LINE_START);
//        auto starting_body = m_startingNode->GetBody();
//        starting_body->AddExternalForce(m_startingForce);
//      }
//
//      if (!m_endingForce) {
//        m_endingForce = std::make_shared<FrCatenaryForce>(GetName() + "_end_force", m_endingNode->GetBody(), this,
//                                                          FrCatenaryLineBase::LINE_END);
//        auto ending_body = m_endingNode->GetBody();
//        ending_body->AddExternalForce(m_endingForce);
//      }
//
//      FrCatenaryAssetOwner::Initialize();
//    }
//
//    FrCable::Initialize();
//  }
//
//  void FrCatenaryLine_::Compute(double time) {
//
//    UpdateTime(time);
//    UpdateState();
//
//  }
//
//  void FrCatenaryLine_::UpdateState() {
//    FrCable::UpdateState();
//    FrCatenaryLine_::solve();
//  }
//
//  void FrCatenaryLine_::StepFinalize() {
//    FrAssetOwner::UpdateAsset();
//    FrPhysicsItem::StepFinalize();
//    FrObject::StepFinalize();
//  }
//
//  internal::FrPhysicsItemBase *FrCatenaryLine_::GetChronoItem_ptr() const { return m_chronoPhysicsItem.get(); }
//
//
  std::shared_ptr<FrCatenaryLine>
  make_catenary_line(const std::string &name,
                     const std::shared_ptr<FrNode> &startingNode,
                     const std::shared_ptr<FrNode> &endingNode,
                     const std::shared_ptr<FrCableProperties> &properties,
                     bool elastic,
                     double unstretchedLength,
                     FLUID_TYPE fluid_type) {

    auto line = std::make_shared<FrCatenaryLine>(name,
                                                 startingNode,
                                                 endingNode,
                                                 properties,
                                                 elastic,
                                                 unstretchedLength,
                                                 fluid_type);
    startingNode->GetBody()->GetSystem()->Add(line);
    return line;

  }

}  // end namespace frydom
