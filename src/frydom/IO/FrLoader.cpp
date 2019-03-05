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


#include "FrLoader.h"

#include "MathUtils/Angles.h"
#include "MathUtils/LookupTable1D.h"

#include "yaml-cpp/yaml.h"



namespace frydom {


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
                angle_unit = mathutils::STRING2ANGLE(node["unit"].as<std::string>());
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
                                    std::vector<std::pair<double, mathutils::Vector3d<double>>>& polar,
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

        std::pair<double, mathutils::Vector3d<double>> new_element;
        for (int i=0; i<angles.size(); i++) {
            polar.push_back( std::pair<double, mathutils::Vector3d<double>>(angles[i], mathutils::Vector3d<double>(cx[i], cy[i], cn[i])));
        }
    }

}  // end namespace frydom
