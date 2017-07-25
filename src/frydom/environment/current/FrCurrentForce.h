//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRCURRENTFORCE_H
#define FRYDOM_FRCURRENTFORCE_H

#include "frydom/core/FrForce.h"
#include "FrCurrentPolarCoeffs.h"

namespace frydom {
namespace environment {

    // Forward declarations
    class FrCurrentPolarCoeffs;
    class FrCurrent;

    class FrCurrentForce : public FrForce {

    private:

        FrCurrentPolarCoeffs coeffs_table;

    public:

        FrCurrentForce() = default;

        explicit FrCurrentForce(std::string yaml_file);

        virtual FrHydroBody* GetBody() { return dynamic_cast<FrHydroBody*>(Body); }

        void UpdateState() override;

        void SetCoeffs(FrCurrentPolarCoeffs table) {
            coeffs_table = std::move(table);
        }

    };





}  // end namespace environment
}  // end namespace frydom

#endif //FRYDOM_FRCURRENTFORCE_H
