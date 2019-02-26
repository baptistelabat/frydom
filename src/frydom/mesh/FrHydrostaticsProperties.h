//
// Created by frongere on 24/05/18.
//

#ifndef FRYDOM_DICE_HYDROSTATICSPROPERTIES_H
#define FRYDOM_DICE_HYDROSTATICSPROPERTIES_H

#include "MathUtils/MathUtils.h"
#include "FrMesh.h"
#include "FrMeshClipper.h"


namespace frydom {

    struct FrHydrostaticMatrixTensor {

        double K33 = 0.;
        double K34 = 0.;
        double K35 = 0.;
        double K44 = 0.;
        double K45 = 0.;
        double K55 = 0.;

        mathutils::MatrixMN<double> GetHydrostaticMatrix() const {
            mathutils::MatrixMN<double> tensor(3, 3);
            tensor(0, 0) = K33;
            tensor(1, 1) = K44;
            tensor(2, 2) = K55;
            tensor(0, 1) = tensor(1, 0) = K34;
            tensor(0, 2) = tensor(2, 0) = K35;
            tensor(1, 2) = tensor(2, 1) = K45;
            return tensor;
        }

    };

    class FrHydrostaticReporter;

    class FrHydrostaticsProperties {
    private:

        friend class FrHydrostaticReporter;

        mesh::FrMesh m_clippedMesh;

        double m_waterDensity = 1023.;
        double m_gravityAcceleration = 9.81;

        double m_volumeDisplacement = 0.;
        double m_waterPlaneArea = 0.;

        double m_hullWetArea = 0.;

        mathutils::Vector3d<double> m_waterPlaneCenter = {0., 0., 0.};

        mathutils::Vector3d<double> m_buoyancyCenter = {0., 0., 0.};
        mathutils::Vector3d<double> m_centerOfGravity = {0., 0., 0.};

        double m_draught = 0.;
        double m_lengthOverallSubmerged = 0.;
        double m_breadthOverallSubmerged = 0.;
        double m_lengthAtWaterLine = 0.;

        double m_transversalMetacentricRadius = 0.;
        double m_longitudinalMetacentricRadius = 0.;
        double m_transversalMetacentricHeight = 0.;
        double m_longitudinalMetacentricHeight = 0.;

        FrHydrostaticMatrixTensor m_hydrostaticTensor;

    public:

        FrHydrostaticsProperties() : m_waterDensity(1023.), m_gravityAcceleration(9.81) {}

        FrHydrostaticsProperties(double waterDensity, double gravityAcceleration) :
                m_waterDensity(waterDensity),
                m_gravityAcceleration(gravityAcceleration) {}

        void Load(const mesh::FrMesh& mesh, mathutils::Vector3d<double> cog) {
            m_centerOfGravity = cog;
            mesh::MeshClipper clipper;
            m_clippedMesh = clipper(mesh);
            Process();
        }

//        void operator() (const mesh::DMesh& mesh) {
//            Load(mesh);
//        }

        void Process() {
            CalcGeometricProperties();
            CalcHydrostaticProperties();
        }

        const mesh::FrMesh& GetHydrostaticMesh() const { return m_clippedMesh; }

        void CalcGeometricProperties() {
            auto bbox = m_clippedMesh.GetBoundingBox();
            m_draught = fabs(bbox.zmin);
            m_lengthAtWaterLine = bbox.xmax - bbox.xmin; // FIXME: attention, ca ne fonctione pas !! Il faut iterer sur les polygones frontiere pour en tirer la bbox...
            // TODO: terminer !!!

        }

        double CalcHydrostaticProperties() {

            // FIXME : attention, appliquer les corrections sur les integrales afin d'exprimer les quantites au centre de gravite
            // ou a un autre point de notre choix !

            // validation node:
            // emoh donne un resultat different d'un ordre pour K34

            // TODO: Voir la doc DIODORE par rapoprt aux conventions qu'ils ont pour le roll pitch yaw a facon euler et le roll
            // pitch yaw a axe fixe et les relations entre ces angles.

            m_volumeDisplacement = m_clippedMesh.GetVolume();

            // Computing the buoyancy center TODO: faire une fonction !!
            double xb, yb, zb;
            xb = yb = zb = 0.;
            mesh::FrMesh::Normal normal;
            for (mesh::FrMesh::FaceIter f_iter = m_clippedMesh.faces_begin(); f_iter != m_clippedMesh.faces_end(); ++f_iter) {
                normal = m_clippedMesh.normal(*f_iter);
                xb += normal[0] * m_clippedMesh.data(*f_iter).GetSurfaceIntegral(mesh::POLY_X2);
                yb += normal[1] * m_clippedMesh.data(*f_iter).GetSurfaceIntegral(mesh::POLY_Y2);
                zb += normal[2] * m_clippedMesh.data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z2);
            }

            xb /= 2. * m_volumeDisplacement;
            yb /= 2. * m_volumeDisplacement;
            zb /= 2. * m_volumeDisplacement; // FIXME: si on prend une cote de surface de clip non nulle, il faut ajouter la quantite ze**2 * Sf

            m_buoyancyCenter = {xb, yb, zb};


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

        std::string GetReport() const;

        mathutils::MatrixMN<double> GetHydrostaticMatrix() const { // FIXME: ne fonctionne pas
            mathutils::MatrixMN<double> mat(3, 3);
            mat(0, 0) = m_hydrostaticTensor.K33;
            mat(1, 1) = m_hydrostaticTensor.K44;
            mat(2, 2) = m_hydrostaticTensor.K55;
            mat(0, 1) = mat(1, 0) = m_hydrostaticTensor.K34;
            mat(0, 2) = mat(2, 0) = m_hydrostaticTensor.K35;
            mat(1, 2) = mat(2, 1) = m_hydrostaticTensor.K34;
            return mat;
        }

    };


    class FrHydrostaticReporter {
    private:
        fmt::MemoryWriter mw;

    public:
        inline void AddLine(std::string preamble, std::string unit, double value) {
            mw << fmt::format("{:-<50s}>\t{:.5g}\n",
                              fmt::format("{:s} ({:s})", preamble, unit),
                              value);
        }

        inline void AddLine(std::string preamble, std::string unit, mathutils::Vector3d<double> value) {
            mw << fmt::format("{:-<50s}>\t{:.5g}\t{:.5g}\t{:.5g}\n",
                              fmt::format("{:s} ({:s})", preamble, unit),
                              value[0], value[1], value[2]);
        }

        template <typename T = double>
        inline void AddLine(std::string preamble, T value) {
            mw << fmt::format("{:-<50s}>\t{:.5g}\n", preamble, value);
        }

        inline void AddLine(std::string preamble) {
            mw << fmt::format("{}\n", preamble);
        }

        void AddBlankLine() {
            mw << "\n";
        };

        std::string operator() (const FrHydrostaticsProperties& hp) {
            mw.clear();

            AddLine(fmt::format("Hydrostatic report for zg = {} (m)", hp.m_centerOfGravity[2]));
            AddLine("---------------------------------");
            AddBlankLine();

            AddLine("GRAVITY ACCELERATION", "M/S**2", hp.m_gravityAcceleration);
            AddLine("WATER DENSITY", "KG/M**3",       hp.m_waterDensity);
            AddBlankLine();

            AddLine("WATERPLANE AREA", "M**2",     hp.m_waterPlaneArea);
            AddLine("WATERPLANE CENTER", "M",      hp.m_waterPlaneCenter);
            AddLine("HULL WET SURFACE", "M**2",    hp.m_hullWetArea);
            AddLine("VOLUME DISPLACEMENT", "M**3", hp.m_volumeDisplacement);
            AddLine("MASS DISPLACEMENT", "TONS",   hp.m_waterDensity * hp.m_volumeDisplacement * 1e-3);
            AddLine("BUOYANCY CENTER", "M",        hp.m_buoyancyCenter);
            AddLine("CENTER OF GRAVITY", "M",      hp.m_centerOfGravity);
            AddBlankLine();

            AddLine("DRAUGHT", "M", hp.m_draught);
            AddLine("LENGTH OVERALL SUBMERGED");
            AddBlankLine();

            AddLine("TRANSVERSAL METACENTRIC RADIUS", "M",        hp.m_transversalMetacentricRadius);
            AddLine("TRANSVERSAL METACENTRIC HEIGHT (GMx)", "M",  hp.m_transversalMetacentricHeight);
            AddLine("LONGITUDINAL METACENTRIC RADIUS", "M",       hp.m_longitudinalMetacentricRadius);
            AddLine("LONGITUDINAL METACENTRIC HEIGHT (GMy)", "M", hp.m_longitudinalMetacentricHeight);
            AddBlankLine();

            AddLine("HYDROSTATIC STIFFNESS COEFFICIENTS:");
            AddLine("\t(Expressed above the center of gravity, on the free surface)");
            AddLine("K33", "N/M", hp.m_hydrostaticTensor.K33);
            AddLine("K34", "N",   hp.m_hydrostaticTensor.K34);
            AddLine("K35", "N",   hp.m_hydrostaticTensor.K35);
            AddLine("K44", "N.M", hp.m_hydrostaticTensor.K44);
            AddLine("K45", "N.M", hp.m_hydrostaticTensor.K45);
            AddLine("K55", "N.M", hp.m_hydrostaticTensor.K55);

            return mw.str();
        }

    };

    std::string FrHydrostaticsProperties::GetReport() const {
        FrHydrostaticReporter reporter;
        return reporter(*this);
    }


}

#endif //FRYDOM_DICE_HYDROSTATICSPROPERTIES_H
