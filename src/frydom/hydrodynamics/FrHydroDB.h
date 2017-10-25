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

    class FrDB {

    protected:
        unsigned int m_nbForce;
        unsigned int m_nbDOF;

    public:

        FrDB(unsigned int nbForce, unsigned int nbDOF) : m_nbForce(nbForce), m_nbDOF(nbDOF) {}

        void SetNbForce(const unsigned int nbForce) { m_nbForce = nbForce; }

        unsigned int GetNbForce() const { return m_nbForce; }

        void SetNbDOF(const unsigned int nbDOF) { m_nbDOF = nbDOF; }

        unsigned int GetNbDOF() const { return m_nbDOF; }

    };




    class FrFrequencyDomainDB {
    protected:
        double m_minFrequency;
        double m_maxFrequency;
        unsigned int m_nbFrequencies;

        int c_iwcut = -1;

    public:
        FrFrequencyDomainDB() = default;

        double GetMinFrequency() const { return m_minFrequency; }
        double GetMaxFrequency() const { return m_maxFrequency; }
        unsigned int GetNbFrequencies() const { return m_nbFrequencies; }

        void SetCutoffFrequency(const double wc) {
            assert(wc <= m_maxFrequency && wc >= m_minFrequency);
            c_iwcut = (uint)(wc / GetDeltaOmega());  // TODO: verifier
        }

        double GetDeltaOmega() const {
            return (m_maxFrequency - m_minFrequency) / (m_nbFrequencies - 1);
        }

        double GetCutoffFrequency() const {
            if (c_iwcut < 0) {
                return m_maxFrequency;
            }

            double dw = GetDeltaOmega();
            return c_iwcut * dw;
        }

        void NoCutoff() {
            c_iwcut = -1;
        }

        std::vector<double> GetOmega() const {
            return linspace(m_minFrequency, GetCutoffFrequency(), m_nbFrequencies);
        }

        std::vector<double> GetFullOmega() const {
            return linspace(m_minFrequency, m_maxFrequency, m_nbFrequencies);
        }

    };

    class FrHydroDB {

    };


    class FrRadiationIRFDB : FrDB {
    // FIXME: a priori, la matrice est symmetrique, il faut en tenir compte...
    private:
        double m_tf = 0.;
        unsigned int m_nt = 0;

        double m_tCutoff = -1.;  // TODO: voir si on regle les cutoff time collectivement ou pour chaque signal...

        std::vector<std::vector<double>> m_Kt;

    public:

        FrRadiationIRFDB(unsigned int nbForce, unsigned int nbDOF) : FrDB(nbForce, nbDOF) {
            auto nbKernels = nbForce * nbDOF;

            // Memory allocation
            m_Kt.reserve(nbKernels);
        };

        void SetTime(const double tf, const unsigned int nt) {
            assert(tf > 0.);

            m_tf = tf;
            m_nt = nt;

            // Memory allocation for every mode
            auto nbKernels = m_nbForce * m_nbDOF;

            for (unsigned int ielt=0; ielt<nbKernels; ++ielt) {
                auto Ktij = std::vector<double>();
                Ktij.reserve(nt);
                m_Kt.push_back(Ktij);
            }

        }

        void SetKernel(unsigned int ielt, const std::vector<double>& Ktij) {
            if (m_nt == 0) {
                throw "Please set the time before feeding with data";
            }

            assert(Ktij.size() == m_nt);

            m_Kt[ielt].assign(Ktij.begin(), Ktij.end());

        }

        void SetKernel(unsigned int iforce, unsigned int idof,
                       const std::vector<double>& Ktij) {

            auto ielt = iforce * m_nbDOF + idof;
            SetKernel(ielt, Ktij);
        }

        std::vector<double> GetKernel(unsigned int ielt) {
            return m_Kt[ielt];
        }

        std::vector<double> GetKernel(unsigned int iforce, unsigned int idof) const {
            return m_Kt[iforce * m_nbDOF + idof];
        }


    };




    class FrBEMForceMode {

        enum TYPE {
            FORCE,
            MOMENT
        };

    private:
        TYPE m_type;
        Eigen::Vector3d m_direction;
        Eigen::Vector3d m_point;

    public:
        FrBEMForceMode() = default;

        void SetTypeFORCE() {
            m_type = FORCE;
            m_point.setZero();
        }
        void SetTypeMOMENT() { m_type = MOMENT; }

        TYPE GetType() const { return m_type; }

        void SetDirection(Eigen::Vector3d& direction) { m_direction = direction; }
        Eigen::Vector3d GetDirection() const { return m_direction; }

        void SetPoint(Eigen::Vector3d& point) { m_point = point; }
        Eigen::Vector3d GetPoint() const {return m_point; }

    };

    class FrBEMMotionMode {

        enum TYPE {
            TRANSLATION,
            ROTATION
        };

    private:
        TYPE m_type;
        Eigen::Vector3d m_direction;
        Eigen::Vector3d m_point;

    public:
        FrBEMMotionMode() = default;

        void SetTypeTRANSLATION() {
            m_type = TRANSLATION;
            m_point.setZero();
        }

        void SetTypeROTATION() { m_type = ROTATION; }

        TYPE GetType() const { return m_type; }

        void SetDirection(Eigen::Vector3d& direction) { m_direction = direction; }
        Eigen::Vector3d GetDirection() const { return m_direction; }

        void SetPoint(Eigen::Vector3d& point) { m_point = point; }
        Eigen::Vector3d GetPoint() const {return m_point; }

    };


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
