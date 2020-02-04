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

  FrHydrostaticsProperties::FrHydrostaticsProperties(double waterDensity, double gravityAcceleration,
                                                     mesh::FrMesh &clipped_mesh, Position cog, FRAME_CONVENTION fc) :
      m_waterDensity(waterDensity),
      m_gravityAcceleration(gravityAcceleration),
      m_clippedMesh(clipped_mesh),
      m_centerOfGravity(cog) {
    if (IsNED(fc)) {
      internal::SwapFrameConvention(m_centerOfGravity);
    }
  }

  FrHydrostaticsProperties::FrHydrostaticsProperties(double waterDensity, double gravityAcceleration,
                                                     mesh::FrMesh &clipped_mesh, Position cog, Position out,
                                                     FRAME_CONVENTION fc) :
      m_waterDensity(waterDensity),
      m_gravityAcceleration(gravityAcceleration),
      m_clippedMesh(clipped_mesh),
      m_centerOfGravity(cog),
      m_reductionPoint(out) {
    if (IsNED(fc)) {
      internal::SwapFrameConvention(m_centerOfGravity);
      internal::SwapFrameConvention(m_reductionPoint);
    }
  }

  void FrHydrostaticsProperties::ComputeProperties() {
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

    m_volumeDisplacement = m_clippedMesh.GetVolume();

    m_buoyancyCenter = m_clippedMesh.GetCOG();

    // Computing temporaries
    double rg = m_waterDensity * m_gravityAcceleration;

    m_hullWetArea = m_clippedMesh.GetArea();

    double Poly1 = 0, PolyX = 0, PolyY = 0, PolyXY = 0, PolyX2 = 0, PolyY2 = 0;

    for (auto &polygon : m_clippedMesh.GetBoundaryPolygonSet()) {

      //FIXME: to check that the mesh is clipped by a horizontal plane
//      assert(polygon.GetPlane().GetFrame().GetRotation().GetQuaternion().GetRotationMatrix().IsIdentity());

      auto BoundaryPolygonsSurfaceIntegral = polygon.GetSurfaceIntegrals();

      Poly1 += BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_1);
      PolyX += BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_X);
      PolyY += BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_Y);
      PolyXY += BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_XY);
      PolyX2 += BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_X2);
      PolyY2 += BoundaryPolygonsSurfaceIntegral.GetSurfaceIntegral(mesh::POLY_Y2);

      m_hullWetArea -= polygon.GetArea();

    }

    m_waterPlaneArea += Poly1;

    double K33 = rg * Poly1;
    double K34 = rg * PolyY;
    double K35 = -rg * PolyX;
    double K45 = -rg * PolyXY;

    m_waterPlaneCenter = Position(
        -K35 / K33,
        K34 / K33,
        m_clippedMesh.GetBoundingBox().zmax
    );

    // Corrections to express the stiffness matrix in a given point
    K34 += rg * (-m_reductionPoint[1] * Poly1);
    K35 += -rg * (-m_reductionPoint[0] * Poly1);
    K45 +=
        -rg * (-m_reductionPoint[1] * PolyX - m_reductionPoint[0] * PolyY +
               m_reductionPoint[0] * m_reductionPoint[1] * Poly1);

    m_transversalMetacentricRadius +=
        (PolyY2 - 2. * m_reductionPoint[1] * PolyY + m_reductionPoint[1] * m_reductionPoint[1] * Poly1) /
        m_volumeDisplacement;
    m_longitudinalMetacentricRadius +=
        (PolyX2 - 2. * m_reductionPoint[0] * PolyX + m_reductionPoint[0] * m_reductionPoint[0] * Poly1) /
        m_volumeDisplacement;

    double zb_zg = m_buoyancyCenter[2] - m_centerOfGravity[2];

    m_transversalMetacentricHeight = m_transversalMetacentricRadius + zb_zg;
    m_longitudinalMetacentricHeight = m_longitudinalMetacentricRadius + zb_zg;

    double rgV = rg * m_volumeDisplacement;
    double K44 = rgV * m_transversalMetacentricHeight;
    double K55 = rgV * m_longitudinalMetacentricHeight;

    m_hydrostaticMatrix.SetDiagonal(K33, K44, K55);
    m_hydrostaticMatrix.SetNonDiagonal(K34, K35, K45);

  }

  std::string FrHydrostaticsProperties::FrHydrostaticsProperties::GetReport() const {
    FrHydrostaticReporter reporter;
    return reporter(*this);
  }

  FrLinearHydrostaticStiffnessMatrix FrHydrostaticsProperties::GetHydrostaticMatrix() const {
    return m_hydrostaticMatrix;
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
