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

//#include "frydom/core/force/FrForce.h"
//#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"
//
//#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroMapper.h"
//
//#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroDB.h"
//
//// <<<<<<<<<<<<<<<<<<<<<< include refactoring
//
//#include "frydom/hydrodynamics/FrEquilibriumFrame.h"


namespace frydom {

    /**
     * \class FrLinearExcitationForce_
     * \brief Class for computing the linear excitation loads.
     */
    class FrLinearExcitationForce_ : public FrForce_ {

    private:

        std::shared_ptr<FrHydroDB_> m_HDB;
        //TODO: passed the raw to shared ptr, need some modif in the mapper.
        FrEquilibriumFrame_* m_equilibriumFrame;

        std::vector<Eigen::MatrixXcd> m_Fexc;

        Matrix66<std::complex<double>> m_steadyForce;

    public:

        FrLinearExcitationForce_(std::shared_ptr<FrHydroDB_> HDB) : m_HDB(HDB) {};

        void Initialize() override;

        void Update(double time) override;

        void StepFinalize() override;

    };

    std::shared_ptr<FrLinearExcitationForce_>
    make_linear_excitation_force(std::shared_ptr<FrHydroDB_> HDB, std::shared_ptr<FrBody_> body);


}  // end namespace frydom

#endif //FRYDOM_FRLINEAREXCITATIONFORCE_H
