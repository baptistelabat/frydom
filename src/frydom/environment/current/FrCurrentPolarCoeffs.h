//
// Created by frongere on 13/07/17.
//

#ifndef FRYDOM_FRCURRENTPOLARCOEFFS_H
#define FRYDOM_FRCURRENTPOLARCOEFFS_H


#include "frydom/misc/FrLookupTable1D.h"


namespace frydom {
namespace environment {

    class FrCurrentPolarCoeffs : public FrLookupTable1D<double> {

    public:

        void Initialize(const std::vector<double>& angles,
                        const std::vector<double>& cx,
                        const std::vector<double>& cy,
                        const std::vector<double>& cz);

        void Initialize(std::string yaml_file);

        double Eval_cx(double angle) {
            return Eval("cx", angle);
        }

        double Eval_cy(double angle) {
            return Eval("cy", angle);
        }

        double Eval_cz(double angle) {
            return Eval("cz", angle);
        }

    };

}  // end namespace environment
}  // end namespace frydom


#endif //FRYDOM_FRCURRENTPOLARCOEFFS_H
