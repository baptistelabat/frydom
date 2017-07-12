//
// Created by frongere on 11/07/17.
//

#include "frydom/misc/FrLinspace.h"
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

    // Eval of LUT given a serie name and a scalar
    auto res = table.Eval("cx", 80);

    // Eval of LUT on a bad key
    try {
        auto res1 = table.Eval("bad_key", 80);
    } catch (std::out_of_range err) {
        std::cout << "Bad key : OK" << std::endl;
    }

    // Eval of LUT given a serie name and a vector
    auto vector = linspace<double>(1.1, 72., 500);
    auto res2 = table.Eval("cx", vector);

    // Eval of LUT given only a scalar (all series are evaluated and an ordered map id returned with results)
    auto res3 = table.Eval(80.2);

    // Eval of LUT given only a vector
    auto res4 = table.Eval(vector);




    return 0;

}