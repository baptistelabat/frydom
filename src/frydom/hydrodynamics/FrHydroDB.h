//
// Created by frongere on 17/10/17.
//

#ifndef FRYDOM_FRHYDRODB_H
#define FRYDOM_FRHYDRODB_H

#include <vector>
#include "Eigen/Dense"

#include "frydom/misc/FrLinspace.h"
#include "frydom/misc/FrEigen.h"

#include "yaml-cpp/yaml.h"


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

    private:
        double m_tf = 0.;
        unsigned int m_nt = 0;

        double m_tCutoff = -1.;  // TODO: voir si on regle les cutoff time collectivement ou pour chaque signal...

        std::vector<Eigen::VectorXd> m_Kt;

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

            // Memory allocation
            auto nbKernels = m_nbForce * m_nbDOF;
            for (uint i=0; i<nbKernels; ++i) {
                m_Kt[i] = Eigen::VectorXd();
            }
        }

        void SetKernel(unsigned int ielt, const Eigen::VectorXd& Ktij) {
            if (m_nt == 0) {
                throw "Please set the time before feeding with data";
            }

            assert(Ktij.rows() == m_nt);

            m_Kt[ielt] = Ktij;
        }

        void SetKernel(unsigned int iforce, unsigned int idof, const Eigen::VectorXd& Ktij) {

            auto ielt = iforce * m_nbDOF + idof;
            SetKernel(ielt, Ktij);
        }

        Eigen::VectorXd GetKernel(unsigned int ielt) {
            return m_Kt[ielt];
        }

        Eigen::VectorXd GetKernel(unsigned int iforce, unsigned int idof) const {
            return m_Kt[iforce * m_nbDOF + idof];
        }


    };


    FrRadiationIRFDB LoadIRFData(std::string yaml_file, std::string key) {

        YAML::Node data = YAML::LoadFile(yaml_file);

        if (data["key"]) {

            auto node = data["key"];

            auto nbForce = node["nbForce"].as<unsigned int>();
            auto nbDOF = node["nbForce"].as<unsigned int>();
            auto nt = node["nt"].as<unsigned int>();
            auto tf = node["tf"].as<double>();
            auto dataFile = node["dataFile"].as<std::string>();

            // Opening the datafile
            // FIXME --> pour le moment, on stocke les donnees de IRF 




            // Instance of the DB
            auto db = FrRadiationIRFDB(nbForce, nbDOF);

            db.SetTime(tf, nt);


        }




    }





}  // end namespace frydom


#endif //FRYDOM_FRHYDRODB_H
