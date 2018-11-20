//
// Created by frongere on 13/07/17.
//

#include "FrLoader.h"
#include "frydom/environment/current/FrCurrentPolarCoeffs.h"
#include <iostream>
#include <fstream>

#include "MathUtils/MathUtils.h"

#include "yaml-cpp/yaml.h"


namespace frydom {

    //=========================================================================================================
    // IO for the current force modeling
    //=========================================================================================================

    FrCurrentPolarCoeffs MakeCurrentPolarCoeffTable(const std::string& yaml_file) {

        // Loading data from yaml file
        std::vector<double> angles, cx, cy, cz;

        LoadPolarCoeffsFromYaml(yaml_file, angles, cx, cy, cz);

        return MakeCurrentPolarCoeffTable(angles, cx, cy, cz);
    }

    FrCurrentPolarCoeffs MakeCurrentPolarCoeffTable(const std::vector<double> &angles,
                                                    const std::vector<double> &cx,
                                                    const std::vector<double> &cy,
                                                    const std::vector<double> &cz) {
        FrCurrentPolarCoeffs table;
        table.Initialize(angles, cx, cy, cz);

        return table;

    }

    void LoadPolarCoeffsFromYaml(const std::string& yaml_file,
                                 std::vector<double> &angles,
                                 std::vector<double> &cx,
                                 std::vector<double> &cy,
                                 std::vector<double> &cz) {

        YAML::Node data = YAML::LoadFile(yaml_file);  // TODO: throw exception if not found !

        ANGLE_UNIT unit = RAD;

        if (data["PolarCurrentCoeffs"]) {
            auto node = data["PolarCurrentCoeffs"];
            // All 4 angles, cx, cy, and cz must be present in the yaml file into the PolarCurrentCoeffs node.

            try {
                unit = STRING2ANGLE(node["unit"].as<std::string>());
            } catch (YAML::BadConversion& err) {
                std::cout << " warning : unit must be DEG or RAD" << std::endl;
            }

            // Getting angles Node
            try {
                angles = node["angles"].as<std::vector<double>>();
            } catch (YAML::BadConversion& err) {
                throw("Cannot read angles");
            }

            if (unit == RAD) {
                rad2deg(angles);
            }

            // Getting cx Node
            try {
                cx = node["cx"].as<std::vector<double>>();
            } catch (YAML::BadConversion& err) {
                throw("Cannot read cx coefficients");
            }

            // Getting cy Node
            try {
                cy = node["cy"].as<std::vector<double>>();
            } catch (YAML::BadConversion& err) {
                throw("Cannot read cy coefficients");
            }

            // Getting cz Node
            try {
                cz = node["cz"].as<std::vector<double>>();
            } catch (YAML::BadConversion& err) {
                throw("Cannot read cz coefficients");
            }

        } else {
            // TODO: trhow an exception if the node is not present

        }
    }

    //=========================================================================================================
    // IO for the wind force model
    //=========================================================================================================

    void LoadWindTableFromYaml(const std::string& yaml_file,
                               std::vector<double>& angle,
                               std::vector<double>& Cx,
                               std::vector<double>& Cy,
                               std::vector<double>& Cz,
                               ANGLE_UNIT& unit) {

        YAML::Node data = YAML::LoadFile(yaml_file);

        if (data["PolarWindCoeffs"]) {

            auto node = data["PolarWindCoeffs"];

            try {
                unit = STRING2ANGLE(node["unit"].as<std::string>());
            } catch (YAML::BadConversion& err) {
                std::cout << " warning : unit must be DEG or RAD" << std::endl;
            }

            try {
                angle = node["angles"].as<std::vector<double>>();
            } catch (YAML::BadConversion& err) {
                // TODO : throw exception
            }

            try {
                Cx = node["cx"].as<std::vector<double>>();
            } catch (YAML::BadConversion& err) {
                // TODO : throw exception
            }

            try {
                Cy = node["cy"].as<std::vector<double>>();
            } catch (YAML::BadConversion& err) {
                // TODO : throw exception
            }

            try {
                Cz = node["cz"].as<std::vector<double>>();
            } catch (YAML::BadConversion& err) {
                // TODO : throw exception
            }

        }

    }

    LookupTable1D<double> MakeWindPolarCoeffTable(const std::string& yaml_file, ANGLE_UNIT unit) {

        std::vector<double> angles, cx, cy, cz;
        LoadWindTableFromYaml(yaml_file, angles, cx, cy, cz, unit);

        LookupTable1D<double, double> lut;
        lut.SetX(angles);
        lut.AddY("cx", cx);
        lut.AddY("cy", cy);
        lut.AddY("cz", cz);

        return lut;
    }


    /// >>>>>>>>>>>>>>>>>>>>>>>>>< REFACTORING


    void LoadFlowPolarCoeffFromYaml(const std::string& yamlFile,
                                    std::vector<double>& angle,
                                    std::vector<double>& cx,
                                    std::vector<double>& cy,
                                    std::vector<double>& cn,
                                    ANGLE_UNIT& angle_unit,
                                    FRAME_CONVENTION& fc,
                                    DIRECTION_CONVENTION& dc) {

        YAML::Node data = YAML::LoadFile(yamlFile);

        if (data["PolarFlowCoeffs"]) {

            auto node = data["PolarFlowCoeffs"];

            try {
                angle_unit = STRING2ANGLE(node["unit"].as<std::string>());
            } catch (YAML::BadConversion& err) {
                std::cout << " warning : unit must be DEG or RAD" << std::endl;
            }

            try {
                fc = STRING2FRAME(node["FRAME_CONVENTION"].as<std::string>());
            } catch (YAML::BadConversion& err) {
                std::cout << " error : reading frame convention. Must be NWU or NED." << std::endl;
            }

            try {
                dc = STRING2DIRECTION(node["DIRECTION_CONVENTION"].as<std::string>());
            } catch (YAML::BadConversion& err) {
                std::cout << " error : reading direction convention. Must be GOTO or COMEFROM." << std::endl;
            }

            try {
                angle = node["angles"].as<std::vector<double>>();
            } catch (YAML::BadConversion& err) {
                // TODO : throw exception
            }

            try {
                cx = node["cx"].as<std::vector<double>>();
            } catch (YAML::BadConversion& err) {
                // TODO : throw exception
            }

            try {
                cy = node["cy"].as<std::vector<double>>();
            } catch (YAML::BadConversion& err) {
                // TODO : throw exception
            }

            try {
                cn = node["cn"].as<std::vector<double>>();
            } catch (YAML::BadConversion& err) {
                // TODO : throw exception
            }

        }

    }

    void LoadFlowPolarCoeffFromYaml(const std::string& yamlFile,
                                    std::vector<std::pair<double, Vector3d<double>>>& polar,
                                    ANGLE_UNIT& unit,
                                    FRAME_CONVENTION& fc,
                                    DIRECTION_CONVENTION& dc) {

        std::vector<double> angles;
        std::vector<double> cx, cy, cn;

        LoadFlowPolarCoeffFromYaml(yamlFile, angles, cx, cy, cn, unit, fc, dc);

        auto n = angles.size();
        assert(cx.size() == n);
        assert(cy.size() == n);
        assert(cn.size() == n);

        std::pair<double, Vector3d<double>> new_element;
        for (int i=0; i<angles.size(); i++) {
            polar.push_back( std::pair<double, Vector3d<double>>(angles[i], Vector3d<double>(cx[i], cy[i], cn[i])));
        }
    }

}  // end namespace frydom