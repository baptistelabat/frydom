//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRCURRENTFORCE_H
#define FRYDOM_FRCURRENTFORCE_H

#include "frydom/core/FrForce.h"
//#include "FrCurrent.h"

#include "FrCurrentPolarCoeffs.h"

namespace frydom {
namespace environment {

    class FrCurrentPolarCoeffs;

    class FrCurrentForce : public FrForce {

    private:
        FrCurrentPolarCoeffs coeffs_table;

        // TODO: il manque les modeles de force et les pptes geometriques du flotteur

    public:

        FrCurrentForce() : FrForce() {};

        FrCurrentForce(std::string yaml_file);

        void UpdateState();

        void SetCoeffs(FrCurrentPolarCoeffs table) {
            coeffs_table = std::move(table);
        }


//    private:

//        environment::FrCurrent *GetCurrent();

    };





}  // end namespace environment
}  // end namespace frydom

#endif //FRYDOM_FRCURRENTFORCE_H
