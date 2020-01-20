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


  bool FrHydroStaticEquilibrium::Solve(double mass, const Position &COGPosInBody) {

    m_residual.SetNull();

    double rhog =
        m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER) * m_body->GetSystem()->GetGravityAcceleration();
    double mg = mass * m_body->GetSystem()->GetGravityAcceleration();

    m_solution.SetNull();
    FrRotation m_bodyRotation;
    m_bodyRotation.SetNullRotation();
    bool convergence = false;

    while (true) {

      m_body->TranslateInWorld(0., 0., m_solution.at(0), NWU);
      m_bodyRotation.SetCardanAngles_RADIANS(m_solution.at(1), m_solution.at(2), 0., NWU);
      m_body->Rotate(m_bodyRotation);

      event_logger::debug("Hydrostatic equilibrium", "", "iteration : {}", m_iteration);

//      std::cout << "    m_body COG position : ("
//                << m_body->GetCOGPositionInWorld(NWU).GetX() << ","
//                << m_body->GetCOGPositionInWorld(NWU).GetY() << ","
//                << m_body->GetCOGPositionInWorld(NWU).GetZ() << ")"
//                << std::endl;

      m_hydroMesh->Update(0.);

      auto clippedMesh = m_hydroMesh->GetClippedMesh();
      // pour remettre le clipped mesh dans le repère du corps
      clippedMesh.Translate(OpenMesh::VectorT<double, 3>(0., 0., -m_body->GetPosition(NWU).GetZ()));

      if (clippedMesh.GetVolume() <= 1E-6) {
        event_logger::info("Hydrostatic equilibrium", "", "body not in water");
        convergence = false;
        break;
      }

      auto rhog_v = rhog * m_hydroMesh->GetClippedMesh().GetVolume();

      // Compute all hydrostatics properties
      FrHydrostaticsProperties hsp(m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER),
                                   m_body->GetSystem()->GetGravityAcceleration(),
                                   clippedMesh,
                                   COGPosInBody);
      hsp.Process();

      m_residual = {rhog_v - mg,
                    rhog_v * hsp.GetBuoyancyCenter().GetY() - mg * COGPosInBody.GetY(),
                    -rhog_v * hsp.GetBuoyancyCenter().GetX() + mg * COGPosInBody.GetX()};

      // FIXME: scale should not vary during solving
      auto scale = Vector3d<double>(mg, mg * hsp.GetBreadthOverallSubmerged(), mg * hsp.GetLengthOverallSubmerged());

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
        event_logger::info("Hydrostatic equilibrium", "", "convergence at an unstable equilibrium : GMx = {}, GMy = {}",
                           hsp.GetLongitudinalMetacentricHeight(), hsp.GetTransversalMetacentricHeight());
        convergence = true;
        break;
      }

      // Get the stiffness matrix
      auto stiffnessMatrix = hsp.GetHydrostaticMatrix();

      event_logger::debug("Hydrostatic equilibrium", "", "residual : {}", m_residual.cwiseQuotient(scale));

      event_logger::debug("Hydrostatic equilibrium", "", "stiffnessMatrix : {}", stiffnessMatrix);

      m_solution = stiffnessMatrix.LUSolver<Vector3d<double>, Vector3d<double>>(m_residual);

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
    clippedMesh.Translate(OpenMesh::VectorT<double, 3>(0., 0., -m_body->GetPosition(NWU).GetZ()));
    // Compute all hydrostatics properties and files a report
    FrHydrostaticsProperties hsp(m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER),
                                 m_body->GetSystem()->GetGravityAcceleration(),
                                 clippedMesh,
                                 COGPosInBody, refPosInBody, fc);
    hsp.Process();

    return hsp.GetReport();
//    event_logger::info("Hydrostatic equilibrium", "", hsp.GetReport());
  }

  FrHydroMesh *FrHydroStaticEquilibrium::GetHydroMesh() const {
    return m_hydroMesh.get();
  }

  FrHydroStaticEquilibrium
  solve_hydrostatic_equilibrium(const std::shared_ptr<FrBody> &body,
                                const std::string &meshFile,
                                FrFrame meshOffset,
                                double mass, const Position &COGPosInBody) {
    auto staticEquilibrium = FrHydroStaticEquilibrium(body, meshFile, meshOffset);

    staticEquilibrium.Solve(mass, COGPosInBody);

    return staticEquilibrium;

  }
} //end namespace frydom
