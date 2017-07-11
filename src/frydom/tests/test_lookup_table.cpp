//
// Created by frongere on 11/07/17.
//

#include "frydom/misc/FrLookupTable1D.h"
#include "yaml-cpp/yaml.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    // Reading some data around
    YAML::Node data = YAML::LoadFile("../src/frydom/tests/data/PolarCurrentCoeffs.yml");

    std::vector<double> angles;
    std::vector<double> cx, cy, cz;


    if (data["PolarCurrentCoeffs"]) {
        auto node = data["PolarCurrentCoeffs"];
        // All 4 angles, cx, cy, and cz must be present in the yaml file into the PolarCurrentCoeffs node.

        // Getting angles Node
        try {
            angles = node["angles"].as<std::vector<double>>();
        } catch (YAML::BadConversion err) {
            // TODO: throw exception
        }

        // Getting cx Node
        try {
            cx = node["cx"].as<std::vector<double>>();
        } catch (YAML::BadConversion err) {
            // TODO: throw exception
        }

        // Getting cy Node
        try {
            cy = node["cy"].as<std::vector<double>>();
        } catch (YAML::BadConversion err) {
            // TODO: throw exception
        }

        // Getting cz Node
        try {
            cz = node["cz"].as<std::vector<double>>();
        } catch (YAML::BadConversion err) {
            // TODO: throw exception
        }

    } else {
        // TODO: trhow an exception if the node is not present

    }

    // Creating a new Lookup table
    FrLookupTable1D<double> table;

    table.SetInterpolationMethod(LINEAR); // optional (default is linear)

    table.SetX(angles);

    bool is_added;
    is_added = table.AddY("cx", cx);
    is_added = table.AddY("cy", cy);
    is_added = table.AddY("cz", cz);







    return 0;

}