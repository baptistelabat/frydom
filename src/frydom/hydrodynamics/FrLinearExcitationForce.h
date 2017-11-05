//
// Created by frongere on 30/10/17.
//

#ifndef FRYDOM_FRLINEAREXCITATIONFORCE_H
#define FRYDOM_FRLINEAREXCITATIONFORCE_H

#include <frydom/core/FrForce.h>
#include <frydom/environment/waves/FrWaveProbe.h>
#include "frydom/environment/waves/FrWaveField.h"
#include "frydom/core/FrHydroBody.h"
#include "FrHydroDB.h"

//#include "Eigen/Dense"

namespace frydom {

    class FrLinearExcitationForce : public FrForce {

    private:

        std::shared_ptr<FrLinearWaveProbe> m_waveProbe;

        std::vector<std::vector<std::complex<double>>> m_steadyForce;

    public:

        void SetWaveProbe(std::shared_ptr<FrLinearWaveProbe>& waveProbe) { m_waveProbe = waveProbe; }

        void Initialize() {
            // From the waveProbe, we get the waveField
            auto waveField = m_waveProbe->GetWaveField();

            bool steady = true;
            auto cmplxElevation =
                waveField->GetCmplxElevation(m_waveProbe->GetX(), m_waveProbe->GetY(), steady);
            // FIXME: Est-ce que waveProbe ce justifie dans ces conditions ???? --> A priori non :/
            // TODO: avoir un waveProbe.GetComplxElevation a la place !!!

            // Gettting the hydrodynamic body...
            auto body = dynamic_cast<FrHydroBody*>(GetBody())->GetBEMBody();

            // Summing over the directions
            unsigned int nbDir = waveField->GetNbWaveDirections();
            unsigned int nbFreq = waveField->GetNbFrequencies();

            unsigned int nbForceMode = body->GetNbForceMode();

            // FIXME: On ne gere pas correctement ici le fait qu'il y a plusieurs modes de force !!!!!

            // Rappel : Pour l'excitation d'un corps, on a par direction une matrice complexe nbForceMode x nbFreq
            // Steady force reduit juste par rapport aux directions.
            // Strady force est donc une matrice de taille nbForceMode x nbFreq

            // Initializations
            m_steadyForce.clear();
            m_steadyForce.reserve(nbForceMode);
            std::vector<std::complex<double>> freqData;
            freqData.reserve(nbFreq);
            for (unsigned int ifreq=0; ifreq<nbFreq; ++ifreq) {
                freqData.push_back(0.);
            }
            for (unsigned int imode=0; imode<nbForceMode; ++imode) {
                m_steadyForce.push_back(freqData);
            }

            // FIXME: la taille de m_steadyForce n'est pas donnee par la HDB mais par le waveField !!!
            // Pour un wave field regulier, on a nbFreq=1, nbDir=1 (mais on a a priori autant de ddl que dans la bdd bien
            // qu'il serait bon de pouvoir linker les ddls entre le corps hydro et le corps bem...)

            // Multiplying steady complex elevations by complex excitation coefficients and summing over wave directions

            auto waveDirections = waveField->GetWaveDirections(RAD);
            auto waveFrequencies = waveField->GetWavePulsations(RADS);


            // TODO: Ici, on a des directions et frequences de vague qui sont celles precisees dans la modelisation de
            // l'etat de mer mais qui sont a priori decouplees de ce qui se trouve dans la base de donnees hydro.
            // Ce qui prime est ce que l'utilisateur entre en donnes pour le modele d'etat de mer et il ne doit pas
            // etre contraint par la HDB.
            // Il convient donc d'effectuer des interpolations sur les donnees de la HDB.
            // Doit-on prevoir un precalcul des interpolateurs pour tous les coeffs hydro ????

//            Eigen::MatrixXcd Fexc_idir;
//            for (unsigned int idir=0; idir<nbDir; ++idir) {
//                // FIXME: il faut intepoler les donnees de la HDB sur les directions demandees !!!
//                Fexc_idir = body->GetExcitation(idir);
//                for (unsigned int imode=0; imode<nbForceMode; ++imode) {
//                    for (unsigned int ifreq=0; ifreq<nbFreq; ++ifreq) {
////                        std::cout << Fexc_idir(imode, ifreq) << std::endl;
////                        std::cout << cmplxElevation[imode][ifreq] << std::endl;
////                        auto couocu = m_steadyForce[imode];
////                        std::cout << m_steadyForce[imode][ifreq] << std::endl;
//
////                        m_steadyForce[imode][ifreq] += Fexc_idir(imode, ifreq);  // Multiplier par l'elevation complexe...
//
//                        // FIXME: il faut une interpolation ici sur les frequences et sur les directions !!!
////                        m_steadyForce[imode][ifreq] += Fexc_idir(imode)
//
//
//                    }
//                }
//            }




            // FIXME: Probleme, on a pas moyen de remonter au fait qu'un mode de force est un mode lineaire ou angulaire...
            // On ne sait pas classer en force et moment dans la force a appliquer au corps


            return;

        }

        void UpdateState() override {

            // Get the wave elevation
//            auto cmplxElevation = m_waveProbe->GetCmplxElevation();
//
//            return;

        }



    };

}  // end namespace frydom

#endif //FRYDOM_FRLINEAREXCITATIONFORCE_H
