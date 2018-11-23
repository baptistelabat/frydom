//
// Created by frongere on 13/07/17.
//

#ifndef FRYDOM_FRLOADER_H
#define FRYDOM_FRLOADER_H


#include <string>
#include <vector>
//#include "MathUtils/MathUtils.h"

#include "frydom/core/FrConvention.h"
#include "frydom/environment/ocean/current/FrCurrentPolarCoeffs.h"

//using namespace mathutils;  // TODO: mettre LookupTable en forward declaration

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
    LookupTable1D<double, double> MakeWindPolarCoeffTable(const std::string& yaml_file, ANGLE_UNIT unit);


    /// >>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

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
                                    std::vector<std::pair<double, Vector3d<double>>>& polar,
                                    ANGLE_UNIT& unit,
                                    FRAME_CONVENTION& fc,
                                    DIRECTION_CONVENTION& dc);


}  // end namespace frydom


#endif //FRYDOM_FRLOADER_H
