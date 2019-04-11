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
#include "frydom/mesh/FrHydroMesh.h"
#include "FrExcitationForceBase.h"

namespace frydom {

    // Forward declarations.
    class FrHydroDB;
    class FrBody;
    class FrEquilibriumFrame;

    /**
     * \class FrNonLinearExcitationForce
     * \brief Class for computing the nonlinear excitation loads (nonlinear FK, linear diffraction).
     */
    class FrNonLinearExcitationForce : public FrExcitationForceBase {

    private:

        /// Offshore system.
        FrOffshoreSystem* m_system;

        /// Clipped mesh.
        mesh::FrMesh m_clipped_mesh;

        /// Free surface object.
        FrFreeSurface* m_free_surface;

        /// Froude-Krylov force;
        Force m_FKforce;

        /// Froude-Krylov torque;
        Torque m_FKtorque;

        /// Hydrodynamic mesh.
        std::shared_ptr<FrHydroMesh> m_hydro_mesh;

    public:

        explicit FrNonLinearExcitationForce(FrOffshoreSystem* system, std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrHydroMesh> HydroMesh) : FrExcitationForceBase(HDB) {
            m_system = system;
            m_hydro_mesh = HydroMesh;
            m_free_surface = m_system->GetEnvironment()->GetOcean()->GetFreeSurface(); // Used to evaluate the incident pressure.
        };

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "NonLinearExcitationForce"; }

        void Initialize() override;

        void StepFinalize() override;

        /// This function compute the incident pressure integration.
        void CalcIncidentPressureIntegration();

        Eigen::MatrixXcd GetHDBData(unsigned int iangle) const override;

        Eigen::VectorXcd GetHDBData(unsigned int iangle, unsigned int iforce) const override;

    private:

        void Compute(double time) override;

    };

    /// This function creates a (fully or weakly) nonlinear excitation force object.
    std::shared_ptr<FrNonLinearExcitationForce>
    make_nonlinear_excitation_force(std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body, std::shared_ptr<FrHydroMesh> HydroMesh);

}  // end namespace frydom

#endif //FRYDOM_FRNONLINEAREXCITATIONFORCE_H
