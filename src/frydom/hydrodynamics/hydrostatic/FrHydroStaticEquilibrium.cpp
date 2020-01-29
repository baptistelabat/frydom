// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source convergence is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

#include "FrHydroStaticEquilibrium.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/core/body/FrInertiaTensor.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/mesh/FrHydroMesh.h"
#include "FrHydrostaticsProperties.h"
#include "frydom/logging/FrEventLogger.h"

namespace frydom {


  FrHydroStaticEquilibrium::FrHydroStaticEquilibrium(std::shared_ptr<FrBody> body, const std::string &meshFile,
                                                     FrFrame meshOffset) :
      m_body(body), m_mass(body->GetMass()), m_COG(body->GetCOG(NWU)),
      m_relax(0.1, 2 * DEG2RAD, 2 * DEG2RAD) {

    // Create a hydroMesh, to set up the mesh in the body frame and then clip it

    m_hydroMesh = make_hydro_mesh("mesh" + body->GetName(),
                                  body,
                                  meshFile,
                                  meshOffset,
                                  FrHydroMesh::ClippingSupport::PLANESURFACE);

    m_hydroMesh->Initialize();
  }


  FrHydroStaticEquilibrium::FrHydroStaticEquilibrium(std::shared_ptr<FrBody> body, const std::string &meshFile,
                                                     FrFrame meshOffset,
                                                     double mass, const Position &COGPosInBody, FRAME_CONVENTION fc) :
      m_body(body), m_mass(mass), m_COG(COGPosInBody),
      m_relax(0.1, 2 * DEG2RAD, 2 * DEG2RAD) {

    if (IsNED(fc)) internal::SwapFrameConvention(m_COG);

    // Create a hydroMesh, to set up the mesh in the body frame and then clip it

    m_hydroMesh = make_hydro_mesh("mesh" + body->GetName(),
                                  body,
                                  meshFile,
                                  meshOffset,
                                  FrHydroMesh::ClippingSupport::PLANESURFACE);

    m_hydroMesh->Initialize();
  }

  FrHydroStaticEquilibrium::~FrHydroStaticEquilibrium() {
    // Remove the temporary m_hydroMesh from the system
    m_body->GetSystem()->Remove(m_hydroMesh);
  }


  void FrHydroStaticEquilibrium::SetMaxIterations(unsigned int max_iterations) {
    m_iterations_max = max_iterations;
  }

  unsigned int FrHydroStaticEquilibrium::GetMaxIterations() const {
    return m_iterations_max;
  }

  void FrHydroStaticEquilibrium::SetRelativeTolerance(double tolerance) {
    m_relative_tolerance = tolerance;
  }

  double FrHydroStaticEquilibrium::GetRelativeTolerance() const {
    return m_relative_tolerance;
  }

  void FrHydroStaticEquilibrium::SetLinearRelaxation(double relaxation) {
    m_relax.at(0) = relaxation;
  }

  void FrHydroStaticEquilibrium::SetAngularRelaxation(double relaxation) {
    m_relax.at(1) = relaxation;
    m_relax.at(2) = relaxation;
  }

  void FrHydroStaticEquilibrium::SetRelaxation(Vector3d<double> &relaxation) {
    m_relax = relaxation;
  }

  Vector3d<double> FrHydroStaticEquilibrium::GetRelaxation() const {
    return m_relax;
  }


  bool FrHydroStaticEquilibrium::SolveDisplacement() {

    double rho = m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
    double g = m_body->GetSystem()->GetGravityAcceleration();
    double mg = m_mass * g;

    double residual;
    double solution;
    bool convergence = false;

    while (true) {

      event_logger::debug("Hydrostatic equilibrium", "", "iteration : {}", m_iteration);

      m_body->TranslateInWorld(0., 0., solution, NWU);

      // Clipping of the mesh, according to the new frame of the body
      m_hydroMesh->Update(0.);

      auto clippedMesh = m_hydroMesh->GetClippedMesh();

      if (clippedMesh.GetVolume() <= 1E-6) {
        event_logger::info("Hydrostatic equilibrium", "", "body not in water");
        convergence = false;
        break;
      }

      // Compute residual
      residual = rho * g * m_hydroMesh->GetClippedMesh().GetVolume() - mg;

      // no convergence reached
      if (m_iteration > m_iterations_max) {
        event_logger::info("Hydrostatic equilibrium", "",
                           "no convergence reached : (residuals: absolute = {}, relative = {})", residual,
                           residual / mg);
        convergence = false;
        break;
      }

      if (abs(residual / mg) < m_relative_tolerance) {
        event_logger::info("Hydrostatic equilibrium", "",
                           "convergence reached in {} iterations : (residuals: absolute = {}, relative = {})",
                           m_iteration, residual, residual / mg);
        convergence = true;
        break;
      }

      // Get the stiffness coefficient and solve the linear system

      auto K33 = 0.;
      for (auto &polygon : clippedMesh.GetBoundaryPolygonSet()) {
        K33 += rho * g * polygon.GetSurfaceIntegrals().GetSurfaceIntegral(mesh::POLY_1);
      }

      solution = residual / K33;

      event_logger::debug("Hydrostatic equilibrium", "", "relative residual : {}", residual / mg);
      event_logger::debug("Hydrostatic equilibrium", "", "K33 : {}", K33);
      event_logger::debug("Hydrostatic equilibrium", "", "solution : {}", solution);

      // Relaxing solution
      if (abs(solution) > m_relax(0)) {
        solution = std::copysign(m_relax(0), solution);
      }

      m_iteration++;

    }

    return convergence;
  }


  bool FrHydroStaticEquilibrium::SolveEquilibrium() {

    SolveDisplacement();

    double rho = m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
    double g = m_body->GetSystem()->GetGravityAcceleration();
    double mg = m_mass * g;

    m_residual.SetNull();
    m_solution.SetNull();
    FrRotation m_bodyRotation;
    m_bodyRotation.SetNullRotation();
    bool convergence = false;

    while (true) {

      event_logger::info("Hydrostatic equilibrium", "", "iteration : {}", m_iteration);

      m_body->TranslateInWorld(0., 0., m_solution.at(0), NWU);
      m_bodyRotation.SetCardanAngles_RADIANS(m_solution.at(1), m_solution.at(2), 0., NWU);
      m_body->Rotate(m_bodyRotation);

      auto COGPosInWorld = m_body->GetPointPositionInWorld(m_COG, NWU);

      // Clipping of the mesh, according to the new frame of the body
      m_hydroMesh->Update(0.);

      auto clippedMesh = m_hydroMesh->GetClippedMesh();
//      clippedMesh.Write("Clipped_Mesh.obj");

      if (clippedMesh.GetVolume() <= 1E-6) {
        event_logger::info("Hydrostatic equilibrium", "", "body not in water");
        convergence = false;
        break;
      }

      // Compute all hydrostatics properties
      FrHydrostaticsProperties hsp(rho, g, clippedMesh, COGPosInWorld, NWU);
      hsp.ComputeProperties();

      auto rhog_v = rho * g * m_hydroMesh->GetClippedMesh().GetVolume();

      m_residual = {rhog_v - mg,
                    rhog_v * hsp.GetBuoyancyCenter().GetY() - mg * COGPosInWorld.GetY(),
                    -rhog_v * hsp.GetBuoyancyCenter().GetX() + mg * COGPosInWorld.GetX()};

      auto scale = Vector3d<double>(mg, mg * hsp.GetBreadthOverallSubmerged(), mg * hsp.GetLengthOverallSubmerged());

      // no convergence reached
      if (m_iteration > m_iterations_max) {
        event_logger::info("Hydrostatic equilibrium", "", "no convergence reached : ({},{},{})", m_residual.at(0),
                           m_residual.at(1), m_residual.at(2));
        convergence = false;
        break;
      }

      if (abs(m_residual.at(0) / scale.at(0)) < m_relative_tolerance and
          abs(m_residual.at(1) / scale.at(1)) < m_relative_tolerance and
          abs(m_residual.at(2) / scale.at(2)) < m_relative_tolerance) {
        // convergence at a stable equilibrium
        if (hsp.GetLongitudinalMetacentricHeight() > 0 and hsp.GetTransversalMetacentricHeight() > 0) {
          event_logger::info("Hydrostatic equilibrium", "", "convergence at a stable equilibrium");
          convergence = true;
          break;
        }
          // convergence at an unstable equilibrium
        else {
          event_logger::info("Hydrostatic equilibrium", "",
                             "convergence at an unstable equilibrium : GMx = {}, GMy = {}",
                             hsp.GetLongitudinalMetacentricHeight(), hsp.GetTransversalMetacentricHeight());
          convergence = true;
          break;
        }
      }

      // Get the stiffness matrix and solve the linear system
      auto stiffnessMatrix = hsp.GetHydrostaticMatrix().GetMatrix();

      m_solution = stiffnessMatrix.LUSolver<Vector3d<double>, Vector3d<double>>(m_residual);

      event_logger::info("Hydrostatic equilibrium", "", "residual : {}", m_residual.cwiseQuotient(scale));
      event_logger::info("Hydrostatic equilibrium", "", "stiffnessMatrix : {}", stiffnessMatrix);
      event_logger::info("Hydrostatic equilibrium", "", "solution : ({},{},{})", m_solution.at(0), m_solution.at(1),
                         m_solution.at(2));

      // Relaxing solution
      for (unsigned int j = 0; j < 3; j++) {
        if (abs(m_solution.at(j)) > m_relax(j)) {
          m_solution.at(j) = std::copysign(m_relax(j), m_solution.at(j));
        }
      }

      m_iteration++;

    }

    return convergence;
  }

  std::string FrHydroStaticEquilibrium::GetReport() const {

    auto COGPosInWorld = m_body->GetPointPositionInWorld(m_COG, NWU);

    return GetReport(COGPosInWorld, NWU);

  }

  std::string FrHydroStaticEquilibrium::GetReport(const Position &reductionPoint, FRAME_CONVENTION fc) const {

    auto COGPosInWorld = m_body->GetPointPositionInWorld(m_COG, NWU);

    // Compute all hydrostatics properties and files a report
    FrHydrostaticsProperties hsp(m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER),
                                 m_body->GetSystem()->GetGravityAcceleration(),
                                 m_hydroMesh->GetClippedMesh(),
                                 COGPosInWorld, reductionPoint, fc);
    hsp.ComputeProperties();

    return hsp.GetReport();

  }

  FrHydroMesh *FrHydroStaticEquilibrium::GetHydroMesh() const {
    return m_hydroMesh.get();
  }

  FrLinearHydrostaticStiffnessMatrix
  FrHydroStaticEquilibrium::GetHydrostaticMatrix() const {

    auto COGPosInWorld = m_body->GetPointPositionInWorld(m_COG, NWU);
    return GetHydrostaticMatrix(COGPosInWorld, NWU);

  }

  FrLinearHydrostaticStiffnessMatrix
  FrHydroStaticEquilibrium::GetHydrostaticMatrix(const Position &reductionPoint, FRAME_CONVENTION fc) const {
    auto COGPosInWorld = m_body->GetPointPositionInWorld(m_COG, NWU);
    // Compute all hydrostatics properties and files a report
    FrHydrostaticsProperties hsp(m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER),
                                 m_body->GetSystem()->GetGravityAcceleration(),
                                 m_hydroMesh->GetClippedMesh(),
                                 COGPosInWorld, reductionPoint, fc);
    hsp.ComputeProperties();

    return hsp.GetHydrostaticMatrix();
  }

  FrHydroStaticEquilibrium
  solve_hydrostatic_equilibrium(const std::shared_ptr<FrBody> &body,
                                const std::string &meshFile,
                                FrFrame meshOffset) {
    auto staticEquilibrium = FrHydroStaticEquilibrium(body, meshFile, meshOffset);

    staticEquilibrium.SolveEquilibrium();

    return staticEquilibrium;

  }

  FrHydroStaticEquilibrium
  solve_hydrostatic_equilibrium(const std::shared_ptr<FrBody> &body,
                                const std::string &meshFile,
                                FrFrame meshOffset,
                                double mass,
                                const Position &COGPosInBody,
                                FRAME_CONVENTION fc) {
    auto staticEquilibrium = FrHydroStaticEquilibrium(body, meshFile, meshOffset, mass, COGPosInBody, fc);

    staticEquilibrium.SolveEquilibrium();

    return staticEquilibrium;

  }
} //end namespace frydom
