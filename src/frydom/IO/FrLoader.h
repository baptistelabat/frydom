//
// Created by frongere on 13/07/17.
//

#ifndef FRYDOM_FRLOADER_H
#define FRYDOM_FRLOADER_H


#include <string>
#include <vector>


namespace frydom {

    class FrCurrentPolarCoeffs;

namespace IO {  // TODO: retirer ce namespace !!

    //=======================================================================================================
    // IO for the current force modeling
    //=======================================================================================================

    /// Build a Polar Coefficient table from a yaml file
    FrCurrentPolarCoeffs MakeCurrentPolarCoeffTable(const std::string yaml_file);


    /// Build a Polar Coefficient table from in-memory coefficient vectors
    FrCurrentPolarCoeffs MakeCurrentPolarCoeffTable(const std::vector<double>& angles,
                                                    const std::vector<double>& cx,
                                                    const std::vector<double>& cy,
                                                    const std::vector<double>& cz);

    /// Extract Polar Coefficient vectors from a yaml file
    void LoadPolarCoeffsFromYaml(const std::string yaml_file,
                                 std::vector<double>& angles,
                                 std::vector<double>& cx,
                                 std::vector<double>& cy,
                                 std::vector<double>& cz);

}  // end namespace IO
}  // end namespace frydom


#endif //FRYDOM_FRLOADER_H
