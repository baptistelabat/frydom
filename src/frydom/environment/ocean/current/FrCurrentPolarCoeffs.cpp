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


#include "FrCurrentPolarCoeffs.h"
#include "frydom/IO/FrLoader.h"

namespace frydom {

    void FrCurrentPolarCoeffs::Initialize(const std::vector<double> &angles,
                                          const std::vector<double> &cx,
                                          const std::vector<double> &cy,
                                          const std::vector<double> &cz) {

        auto n = angles.size();

        assert(cx.size() == n);
        assert(cy.size() == n);
        assert(cz.size() == n);

        SetX(angles);
        AddY("cx", cx);
        AddY("cy", cy);
        AddY("cz", cz);

    }

    void FrCurrentPolarCoeffs::Initialize(std::string yaml_file) {
        std::vector<double> angles, cx, cy, cz;
        LoadPolarCoeffsFromYaml(yaml_file, angles, cx, cy, cz);
        FrCurrentPolarCoeffs::Initialize(angles, cx, cy, cz);
    }

}  // end namespace frydom
