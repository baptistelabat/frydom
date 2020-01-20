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

  FrHydroStaticEquilibrium::FrHydroStaticEquilibrium(std::shared_ptr<FrBody> body,
                                                     std::shared_ptr<FrHydroMesh> hydroMesh)
      : m_body(body),
        m_relax(0.1, 2 * DEG2RAD, 2 * DEG2RAD),
        m_hydroMesh(hydroMesh) {}

  bool FrHydroStaticEquilibrium::Compute() {

    m_residual.SetNull();

    double rhog =
        m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER) * m_body->GetSystem()->GetGravityAcceleration();
    double mg = m_body->GetMass() * m_body->GetSystem()->GetGravityAcceleration();

    m_solution.SetNull();
    FrRotation m_bodyRotation;
    m_bodyRotation.SetNullRotation();
    bool convergence = false;

    while (true) {

      m_body->TranslateInWorld(0., 0., m_solution.at(0), NWU);
      m_bodyRotation.SetCardanAngles_RADIANS(m_solution.at(1), m_solution.at(2), 0., NWU);
      m_body->Rotate(m_bodyRotation);

      std::cout << "iteration : " << m_iteration << std::endl;

      std::cout << "    m_body COG position : ("
                << m_body->GetCOGPositionInWorld(NWU).GetX() << ","
                << m_body->GetCOGPositionInWorld(NWU).GetY() << ","
                << m_body->GetCOGPositionInWorld(NWU).GetZ() << ")"
                << std::endl;

      m_hydroMesh->Update(0.);

      auto clippedMesh = m_hydroMesh->GetClippedMesh();
      // pour remettre le clipped mesh dans le repère du corps
      clippedMesh.Translate(OpenMesh::VectorT<double, 3>(0., 0., -m_body->GetPosition(NWU).GetZ()));

      if (clippedMesh.GetVolume() <= 1E-6) {
        std::cout << "    m_body not in water" << std::endl;
        convergence = false;
        break;
      }

      auto rhog_v = rhog * m_hydroMesh->GetClippedMesh().GetVolume();
      auto m_bodyPosition = m_body->GetPosition(NWU);
      auto m_bodyCOGPosition = m_body->GetCOG(NWU);

      // Compute all hydrostatics properties
      FrHydrostaticsProperties hsp(m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER),
                                   m_body->GetSystem()->GetGravityAcceleration(),
                                   clippedMesh,
                                   m_body->GetCOG(NWU));
      hsp.Process();

      m_residual = {rhog_v - mg,
                    rhog_v * hsp.GetBuoyancyCenter().GetY() - mg * m_bodyCOGPosition.GetY(),
                    -rhog_v * hsp.GetBuoyancyCenter().GetX() + mg * m_bodyCOGPosition.GetX()};

      // FIXME: scale should not vary during solving
      auto scale = Vector3d<double>(mg, mg * hsp.GetBreadthOverallSubmerged(), mg * hsp.GetLengthOverallSubmerged());

      if (m_iteration > m_iterations_max) {
        std::cout << "no convergence reached : (" << m_residual.at(0) << "," << m_residual.at(1) << ","
                  << m_residual.at(2)
                  << ")" << std::endl;
        convergence = false;
        break;
      }

      if (abs(m_residual.at(0) / scale.at(0)) < m_relative_tolerance and
          abs(m_residual.at(1) / scale.at(1)) < m_relative_tolerance and
          abs(m_residual.at(2) / scale.at(2)) < m_relative_tolerance) {
        // convergence at a stable equilibrium
        if (hsp.GetLongitudinalMetacentricHeight() > 0 and hsp.GetTransversalMetacentricHeight() > 0) {
          std::cout << "convergence at a stable equilibrium" << std::endl;
          convergence = true;
          break;
        }

        // convergence at an unstable equilibrium
        std::cout << "convergence at an unstable equilibrium : GMx = " << hsp.GetLongitudinalMetacentricHeight()
                  << ", GMy = " << hsp.GetTransversalMetacentricHeight() << std::endl;
        convergence = true;
        break;
      }

      // Set the stiffness matrix
      auto stiffnessMatrix = hsp.GetHydrostaticMatrix();

      std::cout << "    m_residual : " << m_residual.cwiseQuotient(scale) << std::endl;

      std::cout << "    stiffnessMatrix : " << stiffnessMatrix << std::endl;

      m_solution = stiffnessMatrix.LUSolver<Vector3d<double>, Vector3d<double>>(m_residual);

      std::cout << "    m_solution : (" << m_solution.at(0) << "," << m_solution.at(1) << "," << m_solution.at(2) << ")"
                << std::endl;

      // Relaxing solution
      for (unsigned int j = 0; j < 3; j++) {
        if (abs(m_solution.at(j)) > m_relax(j)) {
          m_solution.at(j) = std::copysign(m_relax(j), m_solution.at(j));
        }
      }

      m_iteration++;

    }

    auto clippedMesh = m_hydroMesh->GetClippedMesh();
    // pour remettre le clipped mesh dans le repère du corps
    clippedMesh.Translate(OpenMesh::VectorT<double, 3>(0., 0., -m_body->GetPosition(NWU).GetZ()));

    // To check the clipped mesh, uncomment the following line
    m_hydroMesh->GetClippedMesh().Write("Clipped_Mesh.obj");

    // Compute all hydrostatics properties and files a report
    FrHydrostaticsProperties hsp(m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER),
                                 m_body->GetSystem()->GetGravityAcceleration(),
                                 clippedMesh,
                                 m_body->GetCOG(NWU), m_body->GetCOG(NWU));
    hsp.Process();
    std::cout << hsp.GetReport() << std::endl;


    // Remove the temporary m_hydroMesh from the system
    m_body->GetSystem()->Remove(m_hydroMesh);

    std::cout << "m_body position : (" << m_body->GetPosition(NWU).GetX() << "," << m_body->GetPosition(NWU).GetY()
              << ","
              << m_body->GetPosition(NWU).GetZ() << ")" << std::endl;

    std::cout << "m_body COG position : ("
              << m_body->GetCOGPositionInWorld(NWU).GetX() << ","
              << m_body->GetCOGPositionInWorld(NWU).GetY() << ","
              << m_body->GetCOGPositionInWorld(NWU).GetZ() << ")"
              << std::endl;

    std::cout << "m_iteration : " << m_iteration << std::endl;

    return convergence;
  }

  FrHydroStaticEquilibrium
  solve_hydrostatic_equilibrium(const std::shared_ptr<FrBody>& body,
                                const std::string &meshFile,
                                FrFrame meshOffset) {
    auto staticEquilibrium = FrHydroStaticEquilibrium(body, meshFile, meshOffset);

    staticEquilibrium.Compute();

    return staticEquilibrium;

  }

  FrHydroStaticEquilibrium
  solve_hydrostatic_equilibrium(const std::shared_ptr<FrBody>& body,const std::shared_ptr<FrHydroMesh>& hydroMesh) {
    auto staticEquilibrium = FrHydroStaticEquilibrium(body, hydroMesh);

    staticEquilibrium.Compute();

    return staticEquilibrium;

  }
} //end namespace frydom
