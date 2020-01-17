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

    Position m_waterPlaneCenter = {0., 0., 0.};

    Position m_buoyancyCenter = {0., 0., 0.};
    Position m_centerOfGravity = {0., 0., 0.};

    double m_draught = 0.;
    double m_lengthOverallSubmerged = 0.;
    double m_breadthOverallSubmerged = 0.;
    double m_lengthAtWaterLine = 0.;

    double m_transversalMetacentricRadius = 0.;
    double m_longitudinalMetacentricRadius = 0.;
    double m_transversalMetacentricHeight = 0.;
    double m_longitudinalMetacentricHeight = 0.;

    FrHydrostaticMatrixTensor m_hydrostaticTensor;

    Position m_outerPoint = {0., 0., 0.};

   public:

    /// Constructors.
    FrHydrostaticsProperties();

    FrHydrostaticsProperties(double waterDensity, double gravityAcceleration);

    FrHydrostaticsProperties(double waterDensity, double gravityAcceleration, mesh::FrMesh &clipped_mesh, Position cog);

    FrHydrostaticsProperties(double waterDensity, double gravityAcceleration, mesh::FrMesh &clipped_mesh, Position cog, Position out);

    /// Compute the geometric and hydrostatic properties
    void Process();

    /// Get the hydrostatic report as a string
    /// \return hydrostatic report
    std::string GetReport() const;

    /// Get the mesh used for the hydrostatic computations
    /// \return mesh used for computations
    const mesh::FrMesh &GetHydrostaticMesh() const;

    /// Get the reduced hydrostatic matrix (K33 to K55)
    /// \return reduced hydrostatic matrix
    mathutils::MatrixMN<double> GetHydrostaticMatrix() const;

    /// Get the tranversal metacentric height
    /// \return tranversal metacentric height
    double GetTransversalMetacentricHeight() const;

    /// Get the longitudinal metacentric height
    /// \return longitudinal metacentric height
    double GetLongitudinalMetacentricHeight() const;

    double GetLengthOverallSubmerged() const {return m_lengthOverallSubmerged;}

    double GetBreadthOverallSubmerged() const {return m_breadthOverallSubmerged;}

    Position GetBuoyancyCenter() const {return m_buoyancyCenter;}

   private:

    /// Compute the geometric properties
    void CalcGeometricProperties();

    /// Compute the hydrostatic properties
    void CalcHydrostaticProperties();

  };

  /**
   * \class FrHydrostaticReporter
   * \brief Class for printing the linear hydrostatic properties.
   */
  class FrHydrostaticReporter {
   private:
    fmt::memory_buffer m_buffer;

   public:
    inline void AddLine(std::string preamble, std::string unit, double value) {
      fmt::format_to(m_buffer, fmt::format("{:-<50s}>\t{:.5g}\n",
                                           fmt::format("{:s} ({:s})", preamble, unit),
                                           value));
    }

    inline void AddLine(std::string preamble, std::string unit, mathutils::Vector3d<double> value) {
      fmt::format_to(m_buffer, fmt::format("{:-<50s}>\t{:.5g}\t{:.5g}\t{:.5g}\n",
                                           fmt::format("{:s} ({:s})", preamble, unit),
                                           value[0], value[1], value[2]));
    }

    template<typename T = double>
    inline void AddLine(std::string preamble, T value) {
      m_buffer << fmt::format("{:-<50s}>\t{:.5g}\n", preamble, value);
    }

    inline void AddLine(std::string preamble) {
      fmt::format_to(m_buffer, "{}\n", preamble);
    }

    void AddBlankLine() {
      fmt::format_to(m_buffer, "\n");
    };

    std::string operator()(const FrHydrostaticsProperties &hp) {
      m_buffer.clear();

      AddLine(fmt::format("Hydrostatic report for zg = {} (m)", hp.m_centerOfGravity[2]));
      AddLine("---------------------------------");
      AddBlankLine();

      AddLine("GRAVITY ACCELERATION", "M/S**2", hp.m_gravityAcceleration);
      AddLine("WATER DENSITY", "KG/M**3", hp.m_waterDensity);
      AddBlankLine();

      AddLine("WATERPLANE AREA", "M**2", hp.m_waterPlaneArea);
      AddLine("WATERPLANE CENTER", "M", hp.m_waterPlaneCenter);
      AddLine("HULL WET SURFACE", "M**2", hp.m_hullWetArea);
      AddLine("VOLUME DISPLACEMENT", "M**3", hp.m_volumeDisplacement);
      AddLine("MASS DISPLACEMENT", "TONS", hp.m_waterDensity * hp.m_volumeDisplacement * 1e-3);
      AddLine("BUOYANCY CENTER", "M", hp.m_buoyancyCenter);
      AddLine("CENTER OF GRAVITY", "M", hp.m_centerOfGravity);
      AddBlankLine();

      AddLine("DRAUGHT", "M", hp.m_draught);
      AddLine("LENGTH OVERALL SUBMERGED", "M", hp.m_lengthOverallSubmerged);
      AddLine("BREADTH OVERALL SUBMERGED", "M", hp.m_breadthOverallSubmerged);
      AddLine("LENGTH AT WATERLINE LWL", "M", hp.m_lengthAtWaterLine);
      AddBlankLine();

      AddLine("TRANSVERSAL METACENTRIC RADIUS", "M", hp.m_transversalMetacentricRadius);
      AddLine("TRANSVERSAL METACENTRIC HEIGHT (GMx)", "M", hp.m_transversalMetacentricHeight);
      AddLine("LONGITUDINAL METACENTRIC RADIUS", "M", hp.m_longitudinalMetacentricRadius);
      AddLine("LONGITUDINAL METACENTRIC HEIGHT (GMy)", "M", hp.m_longitudinalMetacentricHeight);
      AddBlankLine();

      AddLine("HYDROSTATIC STIFFNESS COEFFICIENTS:");
      AddLine("\t(Expressed above the center of gravity, on the free surface)");
      AddLine("K33", "N/M", hp.m_hydrostaticTensor.K33);
      AddLine("K34", "N", hp.m_hydrostaticTensor.K34);
      AddLine("K35", "N", hp.m_hydrostaticTensor.K35);
      AddLine("K44", "N.M", hp.m_hydrostaticTensor.K44);
      AddLine("K45", "N.M", hp.m_hydrostaticTensor.K45);
      AddLine("K55", "N.M", hp.m_hydrostaticTensor.K55);

      return fmt::to_string(m_buffer);
    }

  };


  int solve_hydrostatic_equilibrium(std::shared_ptr<FrBody> body,
                                     const std::string& meshFile,
                                     FrFrame meshOffset);

}

#endif //FRYDOM_DICE_HYDROSTATICSPROPERTIES_H
