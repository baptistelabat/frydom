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
#include "frydom/mesh/FrMesh.h"

namespace frydom {

    class FrEquilibriumFrame;
    class FrBody;
    class FrOffshoreSystem;

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

        /// Offshore system.
        FrOffshoreSystem* m_system;

        /// Hydrodynamic database.
        std::shared_ptr<FrHydroDB> m_HDB;

        /// Input mesh file.
        std::string meshfilename; // Input mesh file.

        /// Center of buoyancy in world.
        Position m_CoBInWorld;

        /// Clipped mesh.
        mesh::FrMesh m_clipped_mesh;

        /// Input mesh file.
        mesh::FrMesh m_mesh_init;

        /// Mesh frame offset in the body frame.
        Position m_MeshOffset;

        /// Rotation of the mesh frame compared to the body frame.
        mathutils::Matrix33<double> m_Rotation;

    public:

        /// Constructor.
        FrWeaklyNonlinearHydrostaticForce(FrOffshoreSystem* system, std::shared_ptr<FrHydroDB> HDB, std::string meshfile) : m_HDB(HDB) {
            m_system = system;
            meshfilename = meshfile;

            // Initilization by default.
            m_Rotation.SetIdentity();
            m_MeshOffset = Position(0,0,0);

        }

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "WeaklyNonlinearHydrostaticForce"; }

        /// Update weakly nonlinear hydrostatic force
        /// \param time Current time of the simulation from begining
        void Update(double time) override;

        /// Intialize the weakly nonlinear hydrostatic force model
        void Initialize() override;

        /// Initialize the log
        void InitializeLog() override;

        /// Methods to be applied at the end of each time steps
        void StepFinalize() override;

        /// This function gives the center of buoyancy.
        Position GetCenterOfBuoyancyInBody(FRAME_CONVENTION fc);

        /// This function sets the offset of the mesh frame in the body frame.
        void SetMeshOffsetRotation(const Position Offset, const mathutils::Matrix33<double> Rotation){
            m_MeshOffset = Offset;
            m_Rotation = Rotation;
        };

    };

    /// This subroutine reads the modes of a body.
    std::shared_ptr<FrWeaklyNonlinearHydrostaticForce>
    make_weakly_nonlinear_hydrostatic_force(FrOffshoreSystem* system,std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body, std::string meshfile);

}  // end namespace frydom


#endif //FRYDOM_FRWEAKLYNONLINEARHYDROSTATICFORCE_H
