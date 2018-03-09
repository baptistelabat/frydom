//
// Created by frongere on 13/07/17.
//

#include "FrLoader.h"
#include "frydom/environment/current/FrCurrentPolarCoeffs.h"
#include <iostream>
#include <fstream>

#include "MathUtils.h"

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

        if (data["PolarCurrentCoeffs"]) {
            auto node = data["PolarCurrentCoeffs"];
            // All 4 angles, cx, cy, and cz must be present in the yaml file into the PolarCurrentCoeffs node.

            // Getting angles Node
            try {
                angles = node["angles"].as<std::vector<double>>();
            } catch (YAML::BadConversion& err) {
                throw("Cannot read angles");
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


}  // end namespace frydom