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


#ifndef FRYDOM_FRWEAKLYNONLINEARHYDROSTATICFORCE_H
#define FRYDOM_FRWEAKLYNONLINEARHYDROSTATICFORCE_H

#include <frydom/core/force/FrForce.h>
#include "frydom/core/math/FrVector.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroDB.h"

namespace frydom {

    class FrEquilibriumFrame;
    class FrBody;

    /// This class defines the weakly nonlinear hydrostatic force applied to a hydrodynamic body.
    /// The force is computed based on the real position of the body with a linearized free surface by integeration of the hydrostatic pressure over the body mesh.
    /// It is supposed that the equilibrium frame has the z-axis pointing upwards and its
    /// position equals the position of the COG of the body at equilibrium

    /**
     * \class FrWeaklyNonlinearHydrostaticForce_
     * \brief Class for computing weakly nonlinear hydrostatic loads.
     */
    class FrWeaklyNonlinearHydrostaticForce : public FrForce {

    private:
        std::shared_ptr<FrHydroDB> m_HDB;
        FrEquilibriumFrame* m_equilibriumFrame;    ///< Equilibrium frame of the body to which the force is applied
        std::string meshfilename; // Input mesh file.

    public:

        /// Constructor.
        FrWeaklyNonlinearHydrostaticForce(std::shared_ptr<FrHydroDB> HDB, std::string meshfile) : m_HDB(HDB) {meshfilename = meshfile; }

        /// Update weakly nonlinear hydrostatic force
        /// \param time Current time of the simulation from begining
        void Update(double time) override;

        /// Intialize the weakly nonlinear hydrostatic force model
        void Initialize() override;

        /// Methods to be applied at the end of each time steps
        void StepFinalize() override;
    };

    /// This subroutine reads the modes of a body.
    std::shared_ptr<FrWeaklyNonlinearHydrostaticForce>
    make_weakly_nonlinear_hydrostatic_force(std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body, std::string meshfile);

}  // end namespace frydom


#endif //FRYDOM_FRWEAKLYNONLINEARHYDROSTATICFORCE_H
