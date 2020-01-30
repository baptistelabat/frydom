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
#include "frydom/logging/FrEventLogger.h"

#include "MathUtils/Angles.h"

#include <nlohmann/json.hpp>

using json = nlohmann::json; // for convenience
#include <fstream>

namespace frydom {

  void LoadFlowPolarCoeffFromJson(const std::string &jsonFile,
                                  std::vector<double> &angle,
                                  std::vector<double> &cx,
                                  std::vector<double> &cy,
                                  std::vector<double> &cn,
                                  double &frontal_area,
                                  double &lateral_area,
                                  double &length,
                                  ANGLE_UNIT &angle_unit,
                                  FRAME_CONVENTION &fc,
                                  DIRECTION_CONVENTION &dc) {

    // This function reads the flow polar coefficients from a Json input file.

    // Loader.
    std::ifstream ifs(jsonFile);
    json j = json::parse(ifs);

    auto node = j["PolarFlowCoeffs"];

    try {
      angle_unit = mathutils::STRING2ANGLE(node["unit"].get<json::string_t>());
    } catch (json::parse_error &err) {
      event_logger::error("FrLoader", "", "Unit must be DEG or RAD in JSON file for polar coefficients");
      exit(EXIT_FAILURE);
    }

    try {
      fc = STRING2FRAME(node["FRAME_CONVENTION"].get<json::string_t>());
    } catch (json::parse_error &err) {
      event_logger::error("FrLoader", "", "reading frame convention. Must be NWU or NED.");
      exit(EXIT_FAILURE);
    }

    try {
      dc = STRING2DIRECTION(node["DIRECTION_CONVENTION"].get<json::string_t>());
    } catch (json::parse_error &err) {
      event_logger::error("FrLoader", "", "reading direction convention. Must be GOTO or COMEFROM.");
      exit(EXIT_FAILURE);
    }

    try {
      frontal_area = node["frontal_area"].get<double>();
    } catch (json::parse_error &err) {
      frontal_area = 1.;
    }

    try {
      lateral_area = node["lateral_area"].get<double>();
    } catch (json::parse_error &err) {
      lateral_area = 1.;
    }

    try {
      length = node["length"].get<double>();
    } catch (json::parse_error &err) {
      length = 1.;
    }

    try {
      angle = node["angles"].get<std::vector<double>>();
    } catch (json::parse_error &err) {
      event_logger::error("FrLoader", "", "JSON file {} parsing error. Could not read {}.",
                          jsonFile, "angles");
      exit(EXIT_FAILURE);
    }

    try {
      cx = node["cx"].get<std::vector<double>>();
    } catch (json::parse_error &err) {
      event_logger::error("FrLoader", "", "JSON file {} parsing error. Could not read {}.",
                          jsonFile, "cx coefficient");
      exit(EXIT_FAILURE);
    }

    try {
      cy = node["cy"].get<std::vector<double>>();
    } catch (json::parse_error &err) {
      event_logger::error("FrLoader", "", "JSON file {} parsing error. Could not read {}.",
                          jsonFile, "cy coefficient");
      exit(EXIT_FAILURE);
    }

    try {
      cn = node["cn"].get<std::vector<double>>();
    } catch (json::parse_error &err) {
      event_logger::error("FrLoader", "", "JSON file {} parsing error. Could not read {}.",
                          jsonFile, "cn coefficient");
      exit(EXIT_FAILURE);
    }

  }

  void LoadFlowPolarCoeffFromJson(const std::string &jsonFile,
                                  std::vector<std::pair<double, mathutils::Vector3d<double>>> &polar,
                                  double &frontal_area,
                                  double &lateral_area,
                                  double &length,
                                  ANGLE_UNIT &unit,
                                  FRAME_CONVENTION &fc,
                                  DIRECTION_CONVENTION &dc) {

    // This function reads the flow polar coefficients from a Json input file.

    std::vector<double> angles;
    std::vector<double> cx, cy, cn;

    // Loading the flow polar coefficients from a json input file.
    LoadFlowPolarCoeffFromJson(jsonFile,
                               angles, cx, cy, cn,
                               frontal_area, lateral_area, length,
                               unit, fc, dc);

    auto n = angles.size();
    assert(cx.size() == n);
    assert(cy.size() == n);
    assert(cn.size() == n);

    std::pair<double, mathutils::Vector3d<double>> new_element;
    for (int i = 0; i < angles.size(); i++) {
      polar.push_back(
          std::pair<double, mathutils::Vector3d<double>>(angles[i], mathutils::Vector3d<double>(cx[i], cy[i], cn[i])));
    }
  }

}  // end namespace frydom
