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
    std::shared_ptr<FrHydroDB> m_hdb;               ///< Hydrodynamic database
    std::shared_ptr<FrWaveDriftPolarData> m_table;  ///< Wave drift coefficient polar table

   public:

    /// Constructor of the wave drift force with specified hydrodynamic database
    /// \param hdb Hydrodynamic database
    FrWaveDriftForce(const std::string &name, FrBody *body, std::shared_ptr<FrHydroDB> hdb);

    /// Method to initialize the wave drift model
    void Initialize() override;

    /// Method to be applied at the end of each time steps
    //void StepFinalize() override;

   private:

    /// Compute the wave drift force
    /// \param time Current time of the simulation from beginning, in seconds
    void Compute(double time) override;

   protected:

    std::vector<double> GetRelativeWaveDir() const;

    std::vector<std::vector<double>> GetEncounterWaveFrequencies(Velocity speed) const;

  };

  std::shared_ptr<FrWaveDriftForce>
  make_wave_drift_force(const std::string &name,
                        std::shared_ptr<FrBody> body,
                        std::shared_ptr<FrHydroDB> HDB);

}  // end namespace frydom

#endif //FRYDOM_FRWAVEDRIFTFORCE_H
