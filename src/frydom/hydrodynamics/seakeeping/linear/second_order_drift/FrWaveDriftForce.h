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


#ifndef FRYDOM_FRWAVEDRIFTFORCE_H
#define FRYDOM_FRWAVEDRIFTFORCE_H

#include <memory>
#include <vector>

#include "frydom/core/force/FrForce.h"



namespace frydom {

    // forward declarations
    class FrHydroDB;
    class FrWaveDriftPolarData;

    /**
    * \class FrWaveDriftForce
    * \brief Class for computing the wave drift force.
    */
    class FrWaveDriftForce : public FrForce {

    private:
        std::shared_ptr<FrHydroDB> m_hdb;
        std::shared_ptr<FrWaveDriftPolarData> m_table;

    public:

        explicit FrWaveDriftForce(std::shared_ptr<FrHydroDB> hdb);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "WaveDriftForce"; }

        void Initialize() override;

        void Update(double time) override;

        void StepFinalize() override;

    protected:

        std::vector<double> GetRelativeWaveDir() const;

        std::vector<std::vector<double>> GetEncounterWaveFrequencies(Velocity speed) const;

    };

}  // end namespace frydom

#endif //FRYDOM_FRWAVEDRIFTFORCE_H
