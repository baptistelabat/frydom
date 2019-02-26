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


#ifndef FRYDOM_FRLOADER_H
#define FRYDOM_FRLOADER_H


#include <string>
#include <vector>
//

#include "MathUtils/LookupTable1D.h"
#include "MathUtils/Vector3d.h"

#include "frydom/core/common/FrConvention.h"
//#include "frydom/environment/ocean/current/FrCurrentPolarCoeffs.h"
#include "frydom/core/common/FrUnits.h"


namespace frydom {

    class FrCurrentPolarCoeffs;

    //=======================================================================================================
    // IO for the current force modeling
    //=======================================================================================================

    /// Build a Polar Coefficient table from a yaml file
    FrCurrentPolarCoeffs MakeCurrentPolarCoeffTable(const std::string& yaml_file);


    /// Build a Polar Coefficient table from in-memory coefficient vectors
    FrCurrentPolarCoeffs MakeCurrentPolarCoeffTable(const std::vector<double>& angles,
                                                    const std::vector<double>& cx,
                                                    const std::vector<double>& cy,
                                                    const std::vector<double>& cz);

    /// Extract Polar Coefficient vectors from a yaml file
    void LoadPolarCoeffsFromYaml(const std::string& yaml_file,
                                 std::vector<double>& angles,
                                 std::vector<double>& cx,
                                 std::vector<double>& cy,
                                 std::vector<double>& cz);

    // =========================================================================================================
    // IO for wind force model
    // =========================================================================================================

    /// Extract coefficients for the wind force model from yaml file
    void LoadWindTableFromYaml(const std::string& yaml_file,
                               std::vector<double>& angle,
                               std::vector<double>& Cx,
                               std::vector<double>& Cy,
                               std::vector<double>& Cz,
                               ANGLE_UNIT& unit);

    /// Build a Polar Coefficient table from a yaml file
    mathutils::LookupTable1D<double, double> MakeWindPolarCoeffTable(const std::string& yaml_file, ANGLE_UNIT unit);


    // >>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    // =========================================================================================================
    // IO for flow force model
    // =========================================================================================================

    /// Extract coefficients for the flow force model from YAML file
    /// \param yamlFile Name of the YAML file containing the polar coefficient
    /// \param angle Flow direction
    /// \param cx Polar coefficient in surge
    /// \param cy Polar coefficient in sway
    /// \param cn Polar coefficient in yaw
    /// \param unit Unit of the angle direction
    void LoadFlowPolarCoeffFromYaml(const std::string& yamlFile,
                                    std::vector<double>& angle,
                                    std::vector<double>& cx,
                                    std::vector<double>& cy,
                                    std::vector<double>& cn,
                                    ANGLE_UNIT& unit,
                                    FRAME_CONVENTION& fc,
                                    DIRECTION_CONVENTION& dc);


    void LoadFlowPolarCoeffFromYaml(const std::string& yamlFile,
                                    std::vector<std::pair<double, mathutils::Vector3d<double>>>& polar,
                                    ANGLE_UNIT& unit,
                                    FRAME_CONVENTION& fc,
                                    DIRECTION_CONVENTION& dc);


}  // end namespace frydom


#endif //FRYDOM_FRLOADER_H
