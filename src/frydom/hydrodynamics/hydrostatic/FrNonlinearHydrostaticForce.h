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


#ifndef FRYDOM_FRNONLINEARHYDROSTATICFORCE_H
#define FRYDOM_FRNONLINEARHYDROSTATICFORCE_H

#include <frydom/core/force/FrForce.h>
#include "frydom/core/math/FrVector.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroDB.h"
#include "frydom/mesh/FrMesh.h"

namespace frydom {

    class FrEquilibriumFrame;
    class FrBody;
    class FrOffshoreSystem;

    /// This class defines the nonlinear hydrostatic force applied to a hydrodynamic body.
    /// The force is computed based on the real position of the body with the incident free surface by integeration of the hydrostatic pressure over the body mesh.
    /// It is supposed that the equilibrium frame has the z-axis pointing upwards and its
    /// position equals the position of the COG of the body at equilibrium

    /**
     * \class FrNonlinearHydrostaticForce_
     * \brief Class for computing nonlinear hydrostatic loads.
     */
    class FrNonlinearHydrostaticForce : public FrForce {

    private:

        /// Offshore system.
        FrOffshoreSystem* m_system;

        /// Hydrodynamic database.
        std::shared_ptr<FrHydroDB> m_HDB;

        /// Input mesh file.
        std::string meshfilename; // Input mesh file.

        /// Center of buoyancy.
        Position m_CoB;

        /// Clipped mesh.
        mesh::FrMesh m_clipped_mesh;

    public:

        /// Constructor.
        FrNonlinearHydrostaticForce(FrOffshoreSystem* system, std::shared_ptr<FrHydroDB> HDB, std::string meshfile) : m_HDB(HDB) {
            m_system = system;
            meshfilename = meshfile;
        }

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "NonlinearHydrostaticForce"; }

        /// Update nonlinear hydrostatic force.
        /// \param time Current time of the simulation from beginning.
        void Update(double time) override;

        /// Intialize the nonlinear hydrostatic force model.
        void Initialize() override;

        /// Initialize the log
        void InitializeLog() override;

        /// Methods to be applied at the end of each time steps.
        void StepFinalize() override;

        /// This function gives the center of buoyancy.
        Position GetCoB(){ return m_CoB;};

    };

    /// This function creates the nonlinear hydrostatic force object for computing the nonlinear hydrostatic loads.
    std::shared_ptr<FrNonlinearHydrostaticForce>
    make_nonlinear_hydrostatic_force(FrOffshoreSystem* system,std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body, std::string meshfile);

}  // end namespace frydom


#endif //FRYDOM_FRNONLINEARHYDROSTATICFORCE_H
