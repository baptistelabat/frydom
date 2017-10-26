//
// Created by frongere on 17/10/17.
//

#ifndef FRYDOM_FRHYDRODB_H
#define FRYDOM_FRHYDRODB_H

#include <vector>
//#include "Eigen/Dense"

#include "frydom/misc/FrLinspace.h"
#include "frydom/misc/FrEigen.h"

#define J std::complex<double>(0, 1)

namespace frydom {


    class FrDiscretization1D {
    private:
        double m_xmin;
        double m_xmax;
        unsigned int m_nx;

    public:
        FrDiscretization1D(double xmin, double xmax, unsigned int nx)
                : m_xmin(xmin), m_xmax(xmax), m_nx(nx) {}

        double GetMin() const { return m_xmin; }
        void SetMin(double xmin) { m_xmin = xmin; }

        double GetMax() const { return m_xmax; }
        void SetMax(double xmax) { m_xmax = xmax; }

        double GetNbSample() const { return m_nx; }
        void SetNbSample(unsigned int nx) { m_nx = nx; }

        std::vector<double> GetVector() const {
            return linspace<double>(m_xmin, m_xmax, m_nx);
        }

    };

//    class FrDB {
//
//    protected:
//        unsigned int m_nbForce;
//        unsigned int m_nbDOF;
//
//    public:
//
//        FrDB(unsigned int nbForce, unsigned int nbDOF) : m_nbForce(nbForce), m_nbDOF(nbDOF) {}
//
//        void SetNbForce(const unsigned int nbForce) { m_nbForce = nbForce; }
//
//        unsigned int GetNbForce() const { return m_nbForce; }
//
//        void SetNbDOF(const unsigned int nbDOF) { m_nbDOF = nbDOF; }
//
//        unsigned int GetNbDOF() const { return m_nbDOF; }
//
//    };

    // FIXME: cette architecture n'est pas la bonne
    // Il faut pour une hdb considerer chaque corps comme appartenant a un groupe de corps en
    // interaction hydrodynamique
    // Chaque objet corps BEM possede des modes de force et de mouvement
    // Chacun de ces modes peut etre actif ou pas --> contraintes de liaison
    // Chque corps BEM possede des coefficients hydrodynamiques frequentiels et des reponses impulsionnelles
    // Pour les excitations, seul le corps concerne subit des efforts suivant a la fois la direction
    // de propagation de la houle et la frequence. Ceci est valable pour la diffraction et Froude-Krylov
    // Pour la radiation, chaque corps est soumis a des efforts suivant ses modes de force en reaction
    // a un mouvement de tous les corps (dont lui-meme) suivant leurs modes de mouvement
    // Pour les modes de mouvement en vitesse, on a de l'amortissement de vague
    // Pour les modes de mouvement en acceleration, on a de la masse ajoutee
    // Pour l'amortissement de vague, on a aussi les reponses impulsionnelles qui donnent la reponse
    // en effort suivant un des modes d'effort du corps considere etant donne un mode de mouvement
    // d'un des corps en interaction (dont le corps subisant l'effort)


//    class FrFrequencyDomainDB {
//    protected:
//        double m_minFrequency;
//        double m_maxFrequency;
//        unsigned int m_nbFrequencies;
//
//        int c_iwcut = -1;
//
//    public:
//        FrFrequencyDomainDB() = default;
//
//        double GetMinFrequency() const { return m_minFrequency; }
//        double GetMaxFrequency() const { return m_maxFrequency; }
//        unsigned int GetNbFrequencies() const { return m_nbFrequencies; }
//
//        void SetCutoffFrequency(const double wc) {
//            assert(wc <= m_maxFrequency && wc >= m_minFrequency);
//            c_iwcut = (uint)(wc / GetDeltaOmega());  // TODO: verifier
//        }
//
//        double GetDeltaOmega() const {
//            return (m_maxFrequency - m_minFrequency) / (m_nbFrequencies - 1);
//        }
//
//        double GetCutoffFrequency() const {
//            if (c_iwcut < 0) {
//                return m_maxFrequency;
//            }
//
//            double dw = GetDeltaOmega();
//            return c_iwcut * dw;
//        }
//
//        void NoCutoff() {
//            c_iwcut = -1;
//        }
//
//        std::vector<double> GetOmega() const {
//            return linspace(m_minFrequency, GetCutoffFrequency(), m_nbFrequencies);
//        }
//
//        std::vector<double> GetFullOmega() const {
//            return linspace(m_minFrequency, m_maxFrequency, m_nbFrequencies);
//        }
//
//    };
//
    class FrHydroDB {

    };
//
//
//    class FrRadiationIRFDB : FrDB {
//    // FIXME: a priori, la matrice est symmetrique, il faut en tenir compte...
//    private:
//        double m_tf = 0.;
//        unsigned int m_nt = 0;
//
//        double m_tCutoff = -1.;  // TODO: voir si on regle les cutoff time collectivement ou pour chaque signal...
//
//        std::vector<std::vector<double>> m_Kt;
//
//    public:
//
//        FrRadiationIRFDB(unsigned int nbForce, unsigned int nbDOF) : FrDB(nbForce, nbDOF) {
//            auto nbKernels = nbForce * nbDOF;
//
//            // Memory allocation
//            m_Kt.reserve(nbKernels);
//        };
//
//        void SetTime(const double tf, const unsigned int nt) {
//            assert(tf > 0.);
//
//            m_tf = tf;
//            m_nt = nt;
//
//            // Memory allocation for every mode
//            auto nbKernels = m_nbForce * m_nbDOF;
//
//            for (unsigned int ielt=0; ielt<nbKernels; ++ielt) {
//                auto Ktij = std::vector<double>();
//                Ktij.reserve(nt);
//                m_Kt.push_back(Ktij);
//            }
//
//        }
//
//        void SetKernel(unsigned int ielt, const std::vector<double>& Ktij) {
//            if (m_nt == 0) {
//                throw "Please set the time before feeding with data";
//            }
//
//            assert(Ktij.size() == m_nt);
//
//            m_Kt[ielt].assign(Ktij.begin(), Ktij.end());
//
//        }
//
//        void SetKernel(unsigned int iforce, unsigned int idof,
//                       const std::vector<double>& Ktij) {
//
//            auto ielt = iforce * m_nbDOF + idof;
//            SetKernel(ielt, Ktij);
//        }
//
//        std::vector<double> GetKernel(unsigned int ielt) {
//            return m_Kt[ielt];
//        }
//
//        std::vector<double> GetKernel(unsigned int iforce, unsigned int idof) const {
//            return m_Kt[iforce * m_nbDOF + idof];
//        }
//
//    };

    class FrBEMMode {

        enum TYPE {
            LINEAR,
            ANGULAR
        };

    private:
        TYPE m_type;
        Eigen::Vector3d m_direction;
        Eigen::Vector3d m_point;
        Eigen::Vector2d vec;

        bool m_active = true;

    public:
        FrBEMMode() = default;

        void SetTypeLINEAR() { m_type = LINEAR; }
        void SetTypeANGULAR() { m_type = ANGULAR; }

        TYPE GetType() const { return m_type; }

        void SetDirection(Eigen::Vector3d& direction) { m_direction = direction; }
        Eigen::Vector3d GetDirection() const { return m_direction; }

        void SetPoint(Eigen::Vector3d& point) { m_point = point; }
        Eigen::Vector3d GetPoint() const {return m_point; }

        // TODO: Ces fonctions sont la pour permettre lors du linkage des corps BEM avec les corps hydro
        // de frydom de supprimer les couplages lorsque par exemple on impose une liaison a un corps
        void Activate() { m_active = true; }
        void Deactivate() { m_active = false; }
        bool IsActive() const { return m_active; }

    };


    typedef FrBEMMode FrBEMForceMode; /// Force modes
    typedef FrBEMMode FrBEMMotionMode; /// Motion modes


    class FrBEMBody {

    private:
        std::vector<FrBEMForceMode> m_ForceModes;
        std::vector<FrBEMMotionMode> m_MotionModes;

        // TODO: maillage du corps a importer egalement

    public:

        unsigned int GetNbForceMode() const { return (uint)m_ForceModes.size(); }
        unsigned int GetNbMotionMode() const { return (uint)m_MotionModes.size(); }

        void AddForceMode(FrBEMForceMode& mode) {
            m_ForceModes.push_back(mode);
        }
        void AddMotionMode(FrBEMMotionMode& mode) {
            m_MotionModes.push_back(mode);
        }



    };






    FrHydroDB LoadHDB5(std::string h5file);



}  // end namespace frydom


#endif //FRYDOM_FRHYDRODB_H
