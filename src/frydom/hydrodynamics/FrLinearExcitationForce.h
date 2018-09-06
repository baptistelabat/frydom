//
// Created by frongere on 30/10/17.
//

#ifndef FRYDOM_FRLINEAREXCITATIONFORCE_H
#define FRYDOM_FRLINEAREXCITATIONFORCE_H

#include "frydom/core/FrForce.h"
#include "frydom/environment/waves/FrWaveProbe.h"
#include "frydom/environment/waves/FrWaveField.h"
#include "frydom/core/FrHydroBody.h"

#include "FrHydroMapper.h"

#include "FrHydroDB.h"


namespace frydom {

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
        void Initialize();  // TODO: devrait s'initialiser automatiquement au lancement de la simulation...

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

}  // end namespace frydom

#endif //FRYDOM_FRLINEAREXCITATIONFORCE_H
