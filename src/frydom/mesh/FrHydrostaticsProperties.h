//
// Created by frongere on 24/05/18.
//

#ifndef FRYDOM_DICE_HYDROSTATICSPROPERTIES_H
#define FRYDOM_DICE_HYDROSTATICSPROPERTIES_H

#include "fmt/format.h"

#include "MathUtils/MathUtils.h"
#include "frydom/mesh/FrMesh.h"
#include "frydom/mesh/FrMeshClipper.h"
#include "frydom/core/math/FrVector.h"

namespace frydom {

    struct FrHydrostaticMatrixTensor {

        double K33 = 0.;
        double K34 = 0.;
        double K35 = 0.;
        double K44 = 0.;
        double K45 = 0.;
        double K55 = 0.;

        mathutils::MatrixMN<double> GetHydrostaticMatrix() const;

    };

    class FrHydrostaticReporter;

    /**
     * \class FrHydrostaticsProperties
     * \brief Class for computing the linear hydrostatic properties.
     */
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

        void Load(const mesh::FrMesh& mesh, mathutils::Vector3d<double> cog);

        void Process();

        const mesh::FrMesh& GetHydrostaticMesh() const { return m_clippedMesh; }

        void CalcGeometricProperties();

        double CalcHydrostaticProperties();

        std::string GetReport() const;

        mathutils::MatrixMN<double> GetHydrostaticMatrix() const;

    };

    /**
     * \class FrHydrostaticReporter
     * \brief Class for printing the linear hydrostatic properties.
     */
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

    /**
     * \class NonlinearHydrostatics
     * \brief Class for computing the hydrostatic pressure integration over a mesh.
     */
    class NonlinearHydrostatics{

    private:

        /// Water density.
        double m_waterDensity = 1023.;

        /// Gravity constant.
        double m_gravityAcceleration = 9.81;

        /// Center of buoyancy in world.
        Position m_centerOfBuoyancy = Position(0.,0.,0.);

        /// Hydrostatic force.
        Force m_force = Force(0.,0.,0.);

    public:

        /// Constructor.
        NonlinearHydrostatics() : m_waterDensity(1023.), m_gravityAcceleration(9.81) {}

        /// Constructor.
        NonlinearHydrostatics(double waterDensity, double gravityAcceleration) :
                m_waterDensity(waterDensity),
                m_gravityAcceleration(gravityAcceleration) {}

        /// This function performs the hydrostatic pressure integration.
        void CalcPressureIntegration(const mesh::FrMesh& clipped_mesh);

        /// This function gives the weakly nonlinear hydrostatic force.
        Force GetWeaklyNonlinearForce(){return m_force;};

        /// This function gives the center of buoyancy of the immersed mesh.
        Position GetCenterOfBuoyancy(){return m_centerOfBuoyancy;};

    };

}

#endif //FRYDOM_DICE_HYDROSTATICSPROPERTIES_H