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


#ifndef FRYDOM_FRNONLINEAREXCITATIONFORCE_H
#define FRYDOM_FRNONLINEAREXCITATIONFORCE_H

#include <memory>
#include <vector>

#include "MathUtils/Matrix66.h"

#include "frydom/core/force/FrForce.h"
#include "frydom/mesh/FrMesh.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/core/FrOffshoreSystem.h"

namespace frydom {

    // Forward declaration
    class FrHydroDB;
    class FrBody;
    class FrEquilibriumFrame;

    /**
     * \class FrNonLinearExcitationForce
     * \brief Class for computing the nonlinear excitation loads (nonlinear FK, linear diffraction).
     */
    class FrNonLinearExcitationForce : public FrForce {

    private:

        /// Offshore system.
        FrOffshoreSystem* m_system;

        std::shared_ptr<FrHydroDB> m_HDB;
        //TODO: passed the raw to shared ptr, need some modif in the mapper.
        FrEquilibriumFrame* m_equilibriumFrame;

        std::vector<Eigen::MatrixXcd> m_Fdiff;

        mathutils::Matrix66<std::complex<double>> m_steadyForce;

        /// Input mesh file.
        std::string meshfilename; // Input mesh file.

        /// Clipped mesh.
        mesh::FrMesh m_clipped_mesh;

        /// Input mesh file.
        mesh::FrMesh m_mesh_init;

        /// Mesh frame offset in the body frame.
        Position m_MeshOffset;

        /// Rotation of the mesh frame compared to the body frame.
        mathutils::Matrix33<double> m_Rotation;

        /// Free surface object.
        FrFreeSurface* m_free_surface;

        /// Froude-Krylov force;
        Force m_FKforce;

        /// Froude-Krylov torque;
        Torque m_FKtorque;

    public:

        explicit FrNonLinearExcitationForce(FrOffshoreSystem* system, std::shared_ptr<FrHydroDB> HDB, std::string meshfile) : m_HDB(HDB) {
            m_system = system;
            meshfilename = meshfile;

            // Initilization by default.
            m_Rotation.SetIdentity();
            m_MeshOffset = Position(0,0,0);
            m_free_surface = m_system->GetEnvironment()->GetOcean()->GetFreeSurface();
        };

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "NonLinearExcitationForce"; }

        void Initialize() override;

        void Update(double time) override;

        void StepFinalize() override;

        /// This function sets the offset of the mesh frame in the body frame.
        void SetMeshOffsetRotation(const Position Offset, const mathutils::Matrix33<double> Rotation){
            m_MeshOffset = Offset;
            m_Rotation = Rotation;
        };

        /// This function compute the incident pressure integration.
        void CalcIncidentPressureIntegration();

    };

    /// This subroutine creates the nonlinear excitation force object.
    std::shared_ptr<FrNonLinearExcitationForce>
    make_nonlinear_excitation_force(FrOffshoreSystem* system,std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body, std::string meshfile);


}  // end namespace frydom

#endif //FRYDOM_FRNONLINEAREXCITATIONFORCE_H
