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
#include "frydom/mesh/FrHydroMesh.h"
#include "frydom/mesh/FrHydrostaticsProperties.h"
#include "frydom/logging/FrEventLogger.h"

namespace frydom {


  FrHydroStaticEquilibrium::FrHydroStaticEquilibrium(std::shared_ptr<FrBody> body, const std::string &meshFile,
                                                     FrFrame meshOffset) : m_body(body),
                                                                           m_relax(0.1, 2 * DEG2RAD, 2 * DEG2RAD) {
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

  double FrHydroStaticEquilibrium::GetRelativeTOlerance() const {
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


  bool FrHydroStaticEquilibrium::Solve(double mass) {
    //TODO: HS equilibrium in displacement first

    double rho = m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
    double g = m_body->GetSystem()->GetGravityAcceleration();
    double mg = mass * g;

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

      // Compute all hydrostatics properties
      FrHydrostaticsProperties hsp(rho, g, clippedMesh, Position(), NWU);
      hsp.Process();

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
                           "convergence reached : (residuals: absolute = {}, relative = {})", residual, residual / mg);
        convergence = true;
        break;
      }

      // Get the stiffness matrix and solve the linear system
      auto stiffnessMatrix = hsp.GetHydrostaticMatrix();

      solution = residual / stiffnessMatrix.at(0, 0);

      event_logger::debug("Hydrostatic equilibrium", "", "relative residual : {}", residual / mg);
      event_logger::debug("Hydrostatic equilibrium", "", "stiffness coefficient : {}", stiffnessMatrix.at(0, 0));
      event_logger::debug("Hydrostatic equilibrium", "", "solution : {}", solution);

      // Relaxing solution
      if (abs(solution) > m_relax(0)) {
        solution = std::copysign(m_relax(0), solution);
      }

      m_iteration++;

    }

    return convergence;
  }


  bool FrHydroStaticEquilibrium::Solve(const FrInertiaTensor &tensor) {


    //TODO: HS equilibrium in displacement first
    Solve(tensor.GetMass());

    double rho = m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
    double g = m_body->GetSystem()->GetGravityAcceleration();
    double mg = tensor.GetMass() * g;

    m_residual.SetNull();
    m_solution.SetNull();
    FrRotation m_bodyRotation;
    m_bodyRotation.SetNullRotation();
    bool convergence = false;

    while (true) {

      event_logger::debug("Hydrostatic equilibrium", "", "iteration : {}", m_iteration);

      m_body->TranslateInWorld(0., 0., m_solution.at(0), NWU);
      m_bodyRotation.SetCardanAngles_RADIANS(m_solution.at(1), m_solution.at(2), 0., NWU);
      m_body->Rotate(m_bodyRotation);

      // Clipping of the mesh, according to the new frame of the body
      m_hydroMesh->Update(0.);

      auto clippedMesh = m_hydroMesh->GetClippedMesh();

      if (clippedMesh.GetVolume() <= 1E-6) {
        event_logger::info("Hydrostatic equilibrium", "", "body not in water");
        convergence = false;
        break;
      }

      // Compute all hydrostatics properties
      FrHydrostaticsProperties hsp(rho, g, clippedMesh, tensor.GetCOGPosition(NWU), NWU);
      hsp.Process();

      auto rhog_v = rho * g * m_hydroMesh->GetClippedMesh().GetVolume();

      m_residual = {rhog_v - mg,
                    rhog_v * hsp.GetBuoyancyCenter().GetY() - mg * tensor.GetCOGPosition(NWU).GetY(),
                    -rhog_v * hsp.GetBuoyancyCenter().GetX() + mg * tensor.GetCOGPosition(NWU).GetX()};

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
        // to transform the clipped mesh back into the body reference frame
        clippedMesh.Translate(OpenMesh::VectorT<double, 3>(0., 0., -m_body->GetPosition(NWU).GetZ()));
        FrHydrostaticsProperties hsp(rho, g, clippedMesh, tensor.GetCOGPosition(NWU), tensor.GetCOGPosition(NWU), NWU);
        hsp.Process();
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
      auto stiffnessMatrix = hsp.GetHydrostaticMatrix();

      m_solution = stiffnessMatrix.LUSolver<Vector3d<double>, Vector3d<double>>(m_residual);

      event_logger::debug("Hydrostatic equilibrium", "", "residual : {}", m_residual.cwiseQuotient(scale));
      event_logger::debug("Hydrostatic equilibrium", "", "stiffnessMatrix : {}", stiffnessMatrix);
      event_logger::debug("Hydrostatic equilibrium", "", "solution : ({},{},{})", m_solution.at(0), m_solution.at(1),
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

  std::string FrHydroStaticEquilibrium::GetReport(const Position &COGPosInBody, const Position &refPosInBody,
                                                  FRAME_CONVENTION fc) const {

    auto clippedMesh = m_hydroMesh->GetClippedMesh();
    // To transform the clipped mesh back into the body reference frame
//    clippedMesh.Translate(OpenMesh::VectorT<double, 3>(0., 0., -m_body->GetPosition(NWU).GetZ()));
    // Compute all hydrostatics properties and files a report
    FrHydrostaticsProperties hsp(m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER),
                                 m_body->GetSystem()->GetGravityAcceleration(),
                                 clippedMesh,
                                 COGPosInBody, refPosInBody, fc);
    hsp.Process();

    return hsp.GetReport();

  }

  FrHydroMesh *FrHydroStaticEquilibrium::GetHydroMesh() const {
    return m_hydroMesh.get();
  }

  FrHydroStaticEquilibrium
  solve_hydrostatic_equilibrium(const std::shared_ptr<FrBody> &body,
                                const std::string &meshFile,
                                FrFrame meshOffset,
                                const FrInertiaTensor &tensor) {
    auto staticEquilibrium = FrHydroStaticEquilibrium(body, meshFile, meshOffset);

    staticEquilibrium.Solve(tensor);

    return staticEquilibrium;

  }
} //end namespace frydom
