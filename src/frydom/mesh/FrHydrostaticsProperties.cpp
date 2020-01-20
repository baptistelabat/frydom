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
                                                     mesh::FrMesh &clipped_mesh, Position cog, Position out, FRAME_CONVENTION fc) :
      m_waterDensity(waterDensity),
      m_gravityAcceleration(gravityAcceleration),
      m_clippedMesh(clipped_mesh),
      m_centerOfGravity(cog),
      m_outerPoint(out) {
    if (IsNED(fc)) {
      internal::SwapFrameConvention(m_centerOfGravity);
      internal::SwapFrameConvention(m_outerPoint);
    }
  }

  void FrHydrostaticsProperties::Process() {
    CalcGeometricProperties();
    CalcHydrostaticProperties();
  }

  void FrHydrostaticsProperties::CalcGeometricProperties() {
    auto bbox = m_clippedMesh.GetBoundingBox();
    m_draught = fabs(bbox.zmax - bbox.zmin);

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

      //FIXME: to check that the mesh is clipped by a horizontal plane
//      assert(polygon.GetPlane().GetFrame().GetRotation().GetQuaternion().GetRotationMatrix().IsIdentity());

      auto BoundaryPolygonsSurfaceIntegral = polygon.GetSurfaceIntegrals();

      m_waterPlaneArea += BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_1);

      m_hydrostaticTensor.K33 += rg * m_waterPlaneArea;
      m_hydrostaticTensor.K34 += rg * (BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_Y));
      m_hydrostaticTensor.K35 += -rg * (BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_X));
      m_hydrostaticTensor.K45 += -rg * (BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_XY));

      m_waterPlaneCenter += Position(
          -m_hydrostaticTensor.K35 / m_hydrostaticTensor.K33,
          m_hydrostaticTensor.K34 / m_hydrostaticTensor.K33,
          m_clippedMesh.GetBoundingBox().zmax
      );

      // Corrections to express the stiffness matrix in a given point
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


}  // end namespace frydom
