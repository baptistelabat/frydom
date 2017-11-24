//
// Created by frongere on 05/07/17.
//

//#include <iostream>
//#include <vector>
//#include <algorithm>
//#include "yaml-cpp/yaml.h"
#include "frydom/frydom.h"

struct PolarCurrentCoeffs {
    std::vector<double> angles;
    std::vector<double> cx;
    std::vector<double> cy;
    std::vector<double> cz;
};

// TODO: A mettre dans les utilitaires...
void to_upper(std::string& mystr) {
    std::transform(mystr.begin(), mystr.end(), mystr.begin(), ::toupper);
}

void to_lower(std::string& mystr) {
    std::transform(mystr.begin(), mystr.end(), mystr.begin(), ::tolower);
}


int main(int argc, char* argv[]) {

    YAML::Node data;
    try {
        data = YAML::LoadFile("../src/frydom/tests/data/PolarCurrentCoeffs.yml");
    } catch (YAML::BadFile err) {
        std::cout << "File not found" << std::endl;
    }

    PolarCurrentCoeffs coeffs;

    if (data["PolarCurrentCoeffs"]) {
        auto node = data["PolarCurrentCoeffs"];


        try {
            coeffs.angles = node["angle"].as<std::vector<double>>();
        } catch (YAML::BadConversion err) {
            std::cout << "angle not found" << std::endl;
        }

        try {
            coeffs.cx = node["cx"].as<std::vector<double>>();
        } catch (YAML::BadConversion err) {
            std::cout << "cx not found" << std::endl;
        }

        try {
            coeffs.cy = node["cy"].as<std::vector<double>>();
        } catch (YAML::BadConversion err) {
            std::cout << "cy not found" << std::endl;
        }

        try {
            coeffs.cz = node["cz"].as<std::vector<double>>();
        } catch (YAML::BadConversion err) {
            std::cout << "cz not found" << std::endl;
        }

        for (int i=0; i < coeffs.angles.size(); i++) {
            std::cout << coeffs.angles[i] << "\t"
                      << coeffs.cx[i] << "\t"
                      << coeffs.cy[i] << "\t"
                      << coeffs.cz[i] << "\t"
                      << std::endl;
        }
    }

}