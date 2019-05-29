//
// Created by frongere on 24/05/18.
//

#include "FrHydrostaticsProperties.h"


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

    void FrHydrostaticsProperties::Process() {
        CalcGeometricProperties();
        CalcHydrostaticProperties();
    }

    void FrHydrostaticsProperties::CalcGeometricProperties() {
        auto bbox = m_clippedMesh.GetBoundingBox();
        m_draught = fabs(bbox.zmin);
        m_lengthAtWaterLine = bbox.xmax - bbox.xmin; // FIXME: attention, ca ne fonctione pas !! Il faut iterer sur les polygones frontiere pour en tirer la bbox...
        // TODO: terminer !!!
    }

    double FrHydrostaticsProperties::CalcHydrostaticProperties() {

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

        m_waterPlaneArea = m_clippedMesh.GetBoundaryPolygonsSurfaceIntegral(mesh::POLY_1);

        m_hydrostaticTensor.K33 =  rg * m_waterPlaneArea;
        m_hydrostaticTensor.K34 =  rg * (m_clippedMesh.GetBoundaryPolygonsSurfaceIntegral(mesh::POLY_Y)
                                         -m_centerOfGravity[1] * m_waterPlaneArea);
        m_hydrostaticTensor.K35 = -rg * (m_clippedMesh.GetBoundaryPolygonsSurfaceIntegral(mesh::POLY_X)
                                         -m_centerOfGravity[0] * m_waterPlaneArea);
        m_hydrostaticTensor.K45 = -rg * (m_clippedMesh.GetBoundaryPolygonsSurfaceIntegral(mesh::POLY_XY)
                                         -m_centerOfGravity[1] * m_clippedMesh.GetBoundaryPolygonsSurfaceIntegral(mesh::POLY_X)
                                         -m_centerOfGravity[0] * m_clippedMesh.GetBoundaryPolygonsSurfaceIntegral(mesh::POLY_Y)
                                         +m_centerOfGravity[0]*m_centerOfGravity[1]*m_waterPlaneArea);

        m_waterPlaneCenter = { // FIXME: valable uniquement avant les corrections precedentes sur le point de calcul !!!
                -m_hydrostaticTensor.K35 / m_hydrostaticTensor.K33,
                m_hydrostaticTensor.K34 / m_hydrostaticTensor.K33,
                0.
        };

        m_transversalMetacentricRadius  =
                (m_clippedMesh.GetBoundaryPolygonsSurfaceIntegral(mesh::POLY_Y2)
                 -2.*m_centerOfGravity[1]*m_clippedMesh.GetBoundaryPolygonsSurfaceIntegral(mesh::POLY_Y)
                 +m_centerOfGravity[1]*m_centerOfGravity[1]*m_waterPlaneArea)
                / m_volumeDisplacement;
        m_longitudinalMetacentricRadius =
                (m_clippedMesh.GetBoundaryPolygonsSurfaceIntegral(mesh::POLY_X2)
                 -2.*m_centerOfGravity[0]*m_clippedMesh.GetBoundaryPolygonsSurfaceIntegral(mesh::POLY_X)
                 +m_centerOfGravity[0]*m_centerOfGravity[0]*m_waterPlaneArea)
                / m_volumeDisplacement;

        double zb_zg = m_buoyancyCenter[2] - m_centerOfGravity[2];

        m_transversalMetacentricHeight  = m_transversalMetacentricRadius  + zb_zg;
        m_longitudinalMetacentricHeight = m_longitudinalMetacentricRadius + zb_zg;

        double rgV = rg * m_volumeDisplacement;
        m_hydrostaticTensor.K44 = rgV * m_transversalMetacentricHeight;
        m_hydrostaticTensor.K55 = rgV * m_longitudinalMetacentricHeight;

        m_hullWetArea = m_clippedMesh.GetArea();

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

}  // end namespace frydom