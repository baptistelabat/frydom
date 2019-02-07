// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#ifndef FRYDOM_FRLINEAREXCITATIONFORCE_H
#define FRYDOM_FRLINEAREXCITATIONFORCE_H

#include "frydom/core/force/FrForce.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveProbe.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"
#include "frydom/core/junk/FrHydroBody.h"

#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroMapper.h"

#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroDB.h"

// <<<<<<<<<<<<<<<<<<<<<< include refactoring

#include "frydom/hydrodynamics/FrEquilibriumFrame.h"

//


namespace frydom {

    /**
     * \class FrLinearExcitationForce
     * \brief Class for computing the linear excitation loads.
     */
    class FrLinearExcitationForce : public FrForce {

    private:

        std::shared_ptr<FrLinearWaveProbe> m_waveProbe; ///< Wave probe attached to the force
        std::vector<Eigen::MatrixXcd> m_Fexc;       ///< Excitation coefficients interpolated from the database
        Eigen::MatrixXcd m_steadyForce;             ///< Steady part of the excitation force
        bool update_position = false;               ///< Moving frame
        int m_HydroMapIndex; //  // TODO : patch hydro map multibody

        /// Return the BEM body linked to the body
        std::shared_ptr<FrBEMBody> GetBEMBody() {
            auto thisHydroBody = dynamic_cast<FrHydroBody*>(GetBody());
            auto BEMBody = dynamic_cast<FrOffshoreSystem*>(GetBody()->GetSystem())->
                    GetHydroMapper(m_HydroMapIndex)->GetBEMBody(thisHydroBody);
            return BEMBody;
        }

    public:

        /// Set the BEM body index in the hydro map
        void SetHydroMapIndex(const int id) { m_HydroMapIndex = id; } // TODO : patch hydro map multibody

        /// Return the BEM body index in the hydro map
        int GetHydroMapIndex() const { return m_HydroMapIndex; } // TODO : patch hydro map multibody

        /// Attached the wave probe
        void SetWaveProbe(std::shared_ptr<FrLinearWaveProbe>& waveProbe) { m_waveProbe = waveProbe; }

        /// Activate / Deactivate position update of the force
        void SetSteady(const bool flag) { update_position = flag; }

        /// Clear steady part of the excitation force
        void Clear() { m_Fexc.clear(); }

        /// Return the complex amplitude of the force
        void GetCmplxForce();

        void SetSteadyForce();

        /// Initialize the steady part of the excitation force
        void Initialize() override;  // TODO: devrait s'initialiser automatiquement au lancement de la simulation...

        /// Update force and moment
        void UpdateState() override;

        void SetLogPrefix(std::string prefix_name) override {
            if (prefix_name=="") {
                m_logPrefix = "Fe_" + FrForce::m_logPrefix;
            } else {
                m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
            }
        }
    };

















    /// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< REFACTORING


    /**
     * \class FrLinearExcitationForce_
     * \brief Class for computing the linear excitation loads.
     */
    class FrLinearExcitationForce_ : public FrForce_ {

    private:

        FrHydroDB_* m_HDB;
        FrEquilibriumFrame_* m_equilibriumFrame;

        std::vector<Eigen::MatrixXcd> m_Fexc;

        Matrix66<std::complex<double>> m_steadyForce;


    public:

        FrLinearExcitationForce_(std::shared_ptr<FrHydroDB_>& HDB, std::shared_ptr<FrEquilibriumFrame_>& eqFrame)
                : m_HDB(HDB.get()), m_equilibriumFrame(eqFrame.get()) {};

        void Initialize() override;

        void Update(double time) override;

        void StepFinalize() override { }

    };


}  // end namespace frydom

#endif //FRYDOM_FRLINEAREXCITATIONFORCE_H
