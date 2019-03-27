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
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/mesh/FrHydroMesh.h"

namespace frydom {

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

        /// Center of buoyancy in world.
        Position m_CoBInWorld;

        /// Clipped mesh.
        mesh::FrMesh m_clipped_mesh;

        /// Hydrodynamic mesh.
        std::shared_ptr<FrHydroMesh> m_hydro_mesh;

    public:

        /// Constructor.
        FrNonlinearHydrostaticForce(FrOffshoreSystem* system, std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrHydroMesh> HydroMesh) : m_HDB(HDB) {
            m_system = system;
            m_hydro_mesh = HydroMesh;
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

        /// This function is called at the end of the time step, after the last step of the integration scheme.
        void StepFinalize() override;

        /// This function gives the center of buoyancy in the world frame.
        Position GetCenterOfBuoyancyInBody(FRAME_CONVENTION fc);

    };

    /// This function creates a (fully or weakly) nonlinear hydrostatic force object.
    std::shared_ptr<FrNonlinearHydrostaticForce>
    make_nonlinear_hydrostatic_force(FrOffshoreSystem* system,std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body, std::shared_ptr<FrHydroMesh> HydroMesh);

}  // end namespace frydom

#endif //FRYDOM_FRNONLINEARHYDROSTATICFORCE_H
