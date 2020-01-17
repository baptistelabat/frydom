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

#include "FrHydrostaticsProperties.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/mesh/FrHydroMesh.h"

namespace frydom {

  mathutils::MatrixMN<double> FrHydrostaticMatrixTensor::GetHydrostaticMatrix() const {
    mathutils::MatrixMN<double> tensor(3, 3);
    tensor(0, 0) = K33;
    tensor(1, 1) = K44;
    tensor(2, 2) = K55;
    tensor(0, 1) = tensor(1, 0) = K34;
    tensor(0, 2) = tensor(2, 0) = K35;
    tensor(1, 2) = tensor(2, 1) = K45;
    return tensor;
  }

  FrHydrostaticsProperties::FrHydrostaticsProperties() : m_waterDensity(1023.), m_gravityAcceleration(9.81) {}

  FrHydrostaticsProperties::FrHydrostaticsProperties(double waterDensity, double gravityAcceleration) :
      m_waterDensity(waterDensity),
      m_gravityAcceleration(gravityAcceleration) {}

  FrHydrostaticsProperties::FrHydrostaticsProperties(double waterDensity, double gravityAcceleration,
                                                     mesh::FrMesh &clipped_mesh, Position cog) :
      m_waterDensity(waterDensity),
      m_gravityAcceleration(gravityAcceleration),
      m_clippedMesh(clipped_mesh),
      m_centerOfGravity(cog) {}

  FrHydrostaticsProperties::FrHydrostaticsProperties(double waterDensity, double gravityAcceleration,
                                                     mesh::FrMesh &clipped_mesh, Position cog, Position out) :
      m_waterDensity(waterDensity),
      m_gravityAcceleration(gravityAcceleration),
      m_clippedMesh(clipped_mesh),
      m_centerOfGravity(cog),
      m_outerPoint(out) {}

  void FrHydrostaticsProperties::Process() {
    CalcGeometricProperties();
    CalcHydrostaticProperties();
  }

  void FrHydrostaticsProperties::CalcGeometricProperties() {
    auto bbox = m_clippedMesh.GetBoundingBox();
    m_draught = fabs(bbox.zmin);

    double xMin, xMax;
    xMin = m_clippedMesh.GetBoundaryPolygonSet()[0].GetBoundingBox().xmin;
    xMax = m_clippedMesh.GetBoundaryPolygonSet()[0].GetBoundingBox().xmax;

    for (auto &polygon : m_clippedMesh.GetBoundaryPolygonSet()) {
      xMin = fmin(xMin, polygon.GetBoundingBox().xmin);
      xMax = fmax(xMax, polygon.GetBoundingBox().xmax);
    }

    m_lengthOverallSubmerged = m_clippedMesh.GetBoundingBox().xmax - m_clippedMesh.GetBoundingBox().xmin;
    m_breadthOverallSubmerged = m_clippedMesh.GetBoundingBox().ymax - m_clippedMesh.GetBoundingBox().ymin;
    m_lengthAtWaterLine = xMax - xMin;


  }

  void FrHydrostaticsProperties::CalcHydrostaticProperties() {

    // FIXME : attention, appliquer les corrections sur les integrales afin d'exprimer les quantites au centre de gravite
    // ou a un autre point de notre choix !

    // validation node:
    // emoh donne un resultat different d'un ordre pour K34

    // TODO: Voir la doc DIODORE par rapoprt aux conventions qu'ils ont pour le roll pitch yaw a facon euler et le roll
    // pitch yaw a axe fixe et les relations entre ces angles.

    m_volumeDisplacement = m_clippedMesh.GetVolume();

    m_buoyancyCenter = m_clippedMesh.GetCOG();

    // Computing temporaries
    double rg = m_waterDensity * m_gravityAcceleration;

    m_hullWetArea = m_clippedMesh.GetArea();

    for (auto &polygon : m_clippedMesh.GetBoundaryPolygonSet()) {

//      assert(polygon.GetPlane().GetFrame().GetRotation().GetQuaternion().GetRotationMatrix().IsIdentity());

      auto BoundaryPolygonsSurfaceIntegral = polygon.GetSurfaceIntegrals();

      m_waterPlaneArea += BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_1);

      m_hydrostaticTensor.K33 += rg * m_waterPlaneArea;
      m_hydrostaticTensor.K34 += rg * (BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_Y));
//                                       - m_centerOfGravity[1] * m_waterPlaneArea);
      m_hydrostaticTensor.K35 += -rg * (BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_X));
//                                        - m_centerOfGravity[0] * m_waterPlaneArea);
      m_hydrostaticTensor.K45 += -rg * (BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_XY));
//                                        - m_centerOfGravity[1] *
//                                          BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_X)
//                                        - m_centerOfGravity[0] *
//                                          BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_Y)
//                                        + m_centerOfGravity[0] * m_centerOfGravity[1] * m_waterPlaneArea);

      m_waterPlaneCenter += Position( // FIXME: valable uniquement avant les corrections precedentes sur le point de calcul !!!
          -m_hydrostaticTensor.K35 / m_hydrostaticTensor.K33,
          m_hydrostaticTensor.K34 / m_hydrostaticTensor.K33,
          0.
      );

      m_hydrostaticTensor.K34 += rg * (- m_outerPoint[1] * m_waterPlaneArea);
      m_hydrostaticTensor.K35 += -rg * (- m_outerPoint[0] * m_waterPlaneArea);
      m_hydrostaticTensor.K45 += -rg * (- m_outerPoint[1] *
                                        BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_X)
                                        - m_outerPoint[0] *
                                        BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_Y)
                                        + m_outerPoint[0] * m_outerPoint[1] * m_waterPlaneArea);

      m_transversalMetacentricRadius +=
          (BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_Y2)
           - 2. * m_outerPoint[1] * BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_Y)
           + m_outerPoint[1] * m_outerPoint[1] * m_waterPlaneArea)
          / m_volumeDisplacement;
      m_longitudinalMetacentricRadius +=
          (BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_X2)
           - 2. * m_outerPoint[0] * BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_X)
           + m_outerPoint[0] * m_outerPoint[0] * m_waterPlaneArea)
          / m_volumeDisplacement;

      m_hullWetArea -= polygon.GetArea();

    }

    double zb_zg = m_buoyancyCenter[2] - m_centerOfGravity[2];

    m_transversalMetacentricHeight = m_transversalMetacentricRadius + zb_zg;
    m_longitudinalMetacentricHeight = m_longitudinalMetacentricRadius + zb_zg;

    double rgV = rg * m_volumeDisplacement;
    m_hydrostaticTensor.K44 = rgV * m_transversalMetacentricHeight;
    m_hydrostaticTensor.K55 = rgV * m_longitudinalMetacentricHeight;

  }

  std::string FrHydrostaticsProperties::FrHydrostaticsProperties::GetReport() const {
    FrHydrostaticReporter reporter;
    return reporter(*this);
  }

  mathutils::MatrixMN<double> FrHydrostaticsProperties::GetHydrostaticMatrix() const {
    mathutils::MatrixMN<double> mat(3, 3);
    mat(0, 0) = m_hydrostaticTensor.K33;
    mat(1, 1) = m_hydrostaticTensor.K44;
    mat(2, 2) = m_hydrostaticTensor.K55;
    mat(0, 1) = mat(1, 0) = m_hydrostaticTensor.K34;
    mat(0, 2) = mat(2, 0) = m_hydrostaticTensor.K35;
    mat(1, 2) = mat(2, 1) = m_hydrostaticTensor.K45;
    return mat;
  }

  const mesh::FrMesh &FrHydrostaticsProperties::GetHydrostaticMesh() const {
    return m_clippedMesh;
  }

  double FrHydrostaticsProperties::GetTransversalMetacentricHeight() const {
    return m_transversalMetacentricHeight;
  }

  double FrHydrostaticsProperties::GetLongitudinalMetacentricHeight() const {
    return m_longitudinalMetacentricHeight;
  }


  int solve_hydrostatic_equilibrium(std::shared_ptr<FrBody> body,
                                    const std::string &meshFile,
                                    FrFrame meshOffset) {

    // Create a hydroMesh, to set up the mesh in the body frame and then clip it
    auto hydroMesh = make_hydro_mesh("mesh" + body->GetName(),
                                     body,
                                     meshFile,
                                     meshOffset,
                                     FrHydroMesh::ClippingSupport::PLANESURFACE);

    hydroMesh->Initialize();

    int itermax = 200;
    double reltol = 1e-2;
    double z_relax = 0.1;
    double thetax_relax = 2*DEG2RAD;
    double thetay_relax = 2*DEG2RAD;

    Vector3d<double> residual;
    residual.SetNull();
    int iter = 0;

    double rhog =
        body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER) * body->GetSystem()->GetGravityAcceleration();
    double mg = body->GetMass() * body->GetSystem()->GetGravityAcceleration();

    Vector3d<double> DZ;
    DZ.SetNull();
    FrRotation bodyRotation;
    bodyRotation.SetNullRotation();
    int code = 0;

    while (true) {

      body->TranslateInWorld(0., 0., DZ.at(0), NWU);
      bodyRotation.SetCardanAngles_RADIANS(DZ.at(1), DZ.at(2), 0., NWU);
      body->Rotate(bodyRotation);

      std::cout<<"iteration : "<<iter<<std::endl;

      std::cout << "    Body COG position : ("
                << body->GetCOGPositionInWorld(NWU).GetX() << ","
                << body->GetCOGPositionInWorld(NWU).GetY() << ","
                << body->GetCOGPositionInWorld(NWU).GetZ() << ")"
                << std::endl;

      hydroMesh->Update(0.);

      if (hydroMesh->GetClippedMesh().GetVolume() <= 1E-6) {
        std::cout<<"    body not in water"<<std::endl;
        break;
      }

      auto rhog_v = rhog * hydroMesh->GetClippedMesh().GetVolume();
      auto bodyPosition = body->GetPosition(NWU);
      auto bodyCOGPosition = body->GetCOG(NWU);

      // Compute all hydrostatics properties
      FrHydrostaticsProperties hsp(body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER),
                                   body->GetSystem()->GetGravityAcceleration(),
                                   hydroMesh->GetClippedMesh(),
                                   body->GetCOGPositionInWorld(NWU));
      hsp.Process();

      residual = {rhog_v - mg,
                  rhog_v * hsp.GetBuoyancyCenter().GetY() - mg * bodyCOGPosition.GetY(),
                  -rhog_v * hsp.GetBuoyancyCenter().GetX() + mg * bodyCOGPosition.GetX()};

      auto scale = Vector3d<double>(mg, mg * hsp.GetBreadthOverallSubmerged(), mg * hsp.GetLengthOverallSubmerged());

      if (iter > itermax) {
        std::cout << "no convergence reached : (" << residual.at(0) << "," << residual.at(1) << "," << residual.at(2)
                  << ")" << std::endl;
        code = 0;
        break;
      }

      if (abs(residual.at(0)/scale.at(0)) < reltol and abs(residual.at(1)/scale.at(1)) < reltol and abs(residual.at(2)/scale.at(2)) < reltol) {
        // convergence at a stable equilibrium
        if (hsp.GetLongitudinalMetacentricHeight() > 0 and hsp.GetTransversalMetacentricHeight() > 0) {
          std::cout << "convergence at a stable equilibrium" << std::endl;
          code = 1;
          break;
        }

        // convergence at an unstable equilibrium
        std::cout << "convergence at an unstable equilibrium : GMx = " << hsp.GetLongitudinalMetacentricHeight()
                  << ", GMy = " << hsp.GetTransversalMetacentricHeight() << std::endl;
        code = 2;
        break;
      }

      // Set the stiffness matrix
      auto stiffnessMatrix = hsp.GetHydrostaticMatrix();

      std::cout<<"    residual : "<<residual.cwiseQuotient(scale)<<std::endl;

      std::cout<<"    stiffnessMatrix : "<<stiffnessMatrix<<std::endl;

      DZ = stiffnessMatrix.LUSolver<Vector3d<double>, Vector3d<double>>(residual);

      std::cout<<"    DZ : ("<<DZ.at(0)<<","<<DZ.at(1)<<","<<DZ.at(2)<<")"<<std::endl;

      if (abs(DZ.at(0)) > z_relax) {
        DZ.at(0) = std::copysign(z_relax,DZ.at(0));
      }
      if (abs(DZ.at(1)) > thetax_relax) {
        DZ.at(1) = std::copysign(thetax_relax,DZ.at(1));
      }
      if (abs(DZ.at(2)) > thetay_relax) {
        DZ.at(2) = std::copysign(thetay_relax,DZ.at(2));
      }


      iter++;

    }

    // Compute all hydrostatics properties and files a report
    FrHydrostaticsProperties hsp(body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER),
                                 body->GetSystem()->GetGravityAcceleration(),
                                 hydroMesh->GetClippedMesh(),
                                 body->GetCOGPositionInWorld(NWU), body->GetCOGPositionInWorld(NWU));
    hsp.Process();
    std::cout << hsp.GetReport() << std::endl;


    // To check the clipped mesh, uncomment the following line
    hydroMesh->GetClippedMesh().Write("Clipped_Mesh.obj");


    // Remove the temporary hydroMesh from the system
    body->GetSystem()->Remove(hydroMesh);

    std::cout << "Body position : (" << body->GetPosition(NWU).GetX() << "," << body->GetPosition(NWU).GetY() << ","
              << body->GetPosition(NWU).GetZ() << ")" << std::endl;

    std::cout << "Body COG position : ("
              << body->GetCOGPositionInWorld(NWU).GetX() << ","
              << body->GetCOGPositionInWorld(NWU).GetY() << ","
              << body->GetCOGPositionInWorld(NWU).GetZ() << ")"
              << std::endl;

    std::cout << "iter : " << iter << std::endl;

    return code;

  }

}  // end namespace frydom
