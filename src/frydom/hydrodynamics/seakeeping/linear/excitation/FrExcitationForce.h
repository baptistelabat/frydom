//// ==========================================================================
//// FRyDoM - frydom-ce.org
////
//// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
//// All rights reserved.
////
//// Use of this source code is governed by a GPLv3 license that can be found
//// in the LICENSE file of FRyDoM.
////
//// ==========================================================================
//
//
//#ifndef FRYDOM_FREXCITATIONFORCE_H
//#define FRYDOM_FREXCITATIONFORCE_H
//
//
//// FIXME : NOT USED ANYMORE, TO DELETE
//
//
//namespace frydom {
//
//    /**
//     * \class FrLinearExcitationForceEncounter
//     * \brief Class not used.
//     */
//    class FrLinearExcitationForceEncounter : public FrForce {
//
//    protected:
//
//        std::shared_ptr<FrWaveProbe> m_waveProbe;                   ///< Wave probe linked to the wave field
//        std::vector<Eigen::MatrixXcd> m_Fexc;
//        int m_HydroMapIndex;
//
//
//        std::shared_ptr<FrBEMBody> GetBEMBody() {
//            auto thisHydroBody = dynamic_cast<FrHydroBody*>(GetBody());
//            auto BEMBody = dynamic_cast<FrOffshoreSystem*>(GetBody()->GetSystem())->
//                    GetHydroMapper(m_HydroMapIndex)->GetBEMBody(thisHydroBody);
//            return BEMBody;
//        }
//
//    public:
//
//        /// Set hydro map index corresponding to the BEMBody id to which the body is linked
//        void SetHydroMapIndex(const int id) { m_HydroMapIndex = id; } // TODO : patch hydro map multibody
//
//        /// Get BEMBody index in hydro map
//        int GetHydroMapIndex() const { return m_HydroMapIndex; } // TODO : patch hydro map multibody
//
//        /// Set wave probe adapted for linear wave with encounter frequency
//        void SetWaveProbe(std::shared_ptr<FrLinearWaveProbeEncounter>& waveProbe) { m_waveProbe = waveProbe; }
//
//        /// Clear the excitation force vector
//        void Clear() {
//            m_Fexc.clear();
//        }
//
//        /// Initialize the excitation force vector
//        void Initialize() {
//
//            auto BEMBody = GetBEMBody();
//
//            auto waveField = m_waveProbe->GetWaveField();
//
//            if (m_Fexc.empty()) {
//                // We initialize the Fexc coefficients by interpolation on the Hydrodynamic Database
//                m_Fexc = BEMBody->GetExcitationInterp(waveField->GetWaveFrequencies(RADS),
//                                                      waveField->GetWaveDirections(DEG),
//                                                      DEG);
//            }
//        }
//
//        void UpdateState() override {
//
//            auto BEMBody = GetBEMBody();
//
//            auto nbFreq = m_waveProbe->GetWaveField()->GetNbFrequencies();
//            auto nbMode = BEMBody->GetForceMode();
//            auto nbDir = m_waveProbe->GetWaveField()->GetNbDirection();
//
//            Eigen::VectorXd forceMode(nbMode);
//            forceMode.SetZero();
//
//            auto cmplxElevation = m_waveProbe->GetCmplxElevation();
//
//            for (unsigned int imode=0; imode<nbMode; ++imode) {
//                for (unsigned int ifreq=0; ifreq<nbFreq; ++ifreq) {
//                    for (unsigned int idir=0; idir<nbWaveDir; ++idir) {
//                        forceMode(imode) += std::imag(cmplxElevation[idir][ifreq] * m_Fexc[idir](imode, ifreq));
//                    }
//                }
//            }
//
//        }
//
//
//    };
//
//
//}
//
//#endif //FRYDOM_FREXCITATIONFORCE_H
