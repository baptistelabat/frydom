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


#ifndef FRYDOM_FRLINEAREXCITATIONFORCE_H
#define FRYDOM_FRLINEAREXCITATIONFORCE_H

#include <memory>
#include <vector>

#include "MathUtils/Matrix66.h"

#include "frydom/core/force/FrForce.h"



namespace frydom {

    // Forward declaration
    class FrHydroDB;
    class FrBody;
    class FrEquilibriumFrame;



    /**
     * \class FrLinearExcitationForce_
     * \brief Class for computing the linear excitation loads.
     */
    class FrLinearExcitationForce : public FrForce {

    private:

        std::shared_ptr<FrHydroDB> m_HDB;
        //TODO: passed the raw to shared ptr, need some modif in the mapper.
        FrEquilibriumFrame* m_equilibriumFrame;

        std::vector<Eigen::MatrixXcd> m_Fexc;

        mathutils::Matrix66<std::complex<double>> m_steadyForce;

    public:

        explicit FrLinearExcitationForce(std::shared_ptr<FrHydroDB> HDB) : m_HDB(HDB) {};

        void Initialize() override;

        void Update(double time) override;

        void StepFinalize() override;

    };

    std::shared_ptr<FrLinearExcitationForce>
    make_linear_excitation_force(std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body);


}  // end namespace frydom

#endif //FRYDOM_FRLINEAREXCITATIONFORCE_H
