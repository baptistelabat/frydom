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

#include "frydom/frydom.h"

struct PolarCurrentCoeffs {
  std::vector<double> angles;
  std::vector<double> cx;
  std::vector<double> cy;
  std::vector<double> cz;
};

// TODO: A mettre dans les utilitaires...
void to_upper(std::string &mystr) {
  std::transform(mystr.begin(), mystr.end(), mystr.begin(), ::toupper);
}

void to_lower(std::string &mystr) {
  std::transform(mystr.begin(), mystr.end(), mystr.begin(), ::tolower);
}


int main(int argc, char *argv[]) {

  YAML::Node data;
  try {
    data = YAML::LoadFile("PolarCurrentCoeffs.yml");
  } catch (YAML::BadFile &err) {
    std::cout << "File not found" << std::endl;
  }

  PolarCurrentCoeffs coeffs;

  if (data["PolarCurrentCoeffs"]) {
    auto node = data["PolarCurrentCoeffs"];


    try {
      coeffs.angles = node["angles"].as<std::vector<double>>();
    } catch (YAML::BadConversion &err) {
      std::cout << "angles not found" << std::endl;
    }

    try {
      coeffs.cx = node["cx"].as<std::vector<double>>();
    } catch (YAML::BadConversion &err) {
      std::cout << "cx not found" << std::endl;
    }

    try {
      coeffs.cy = node["cy"].as<std::vector<double>>();
    } catch (YAML::BadConversion &err) {
      std::cout << "cy not found" << std::endl;
    }

    try {
      coeffs.cz = node["cz"].as<std::vector<double>>();
    } catch (YAML::BadConversion &err) {
      std::cout << "cz not found" << std::endl;
    }

    for (int i = 0; i < coeffs.angles.size(); i++) {
      std::cout << coeffs.angles[i] << "\t"
                << coeffs.cx[i] << "\t"
                << coeffs.cy[i] << "\t"
                << coeffs.cz[i] << "\t"
                << std::endl;
    }
  }

}