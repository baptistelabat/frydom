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

#include <frydom/core/force/FrForce.h>
#include <frydom/environment/ocean/freeSurface/waves/FrWaveProbe.h>
#include <frydom/core/junk/FrHydroBody.h>
#include <MathUtils/MathUtils.h>

namespace frydom {

    // forward declarations
    class FrHydroDB_;
    class FrWaveDriftPolarData;

    /**
    * \class FrWaveDriftForce_
    * \brief Class for computing the wave drift force.
    */
    class FrWaveDriftForce_ : public FrForce_ {

    private:
        std::shared_ptr<FrHydroDB_> m_hdb;
        std::shared_ptr<FrWaveDriftPolarData> m_table;

    public:

        FrWaveDriftForce_(std::shared_ptr<FrHydroDB_> hdb);

        void Initialize() override;

        void Update(double time) override;

        void StepFinalize() override;

    protected:

        std::vector<double> GetRelativeWaveDir() const;

        std::vector<std::vector<double>> GetEncounterWaveFrequencies(Velocity speed) const;

    };

}  // end namespace frydom

#endif //FRYDOM_FRWAVEDRIFTFORCE_H
