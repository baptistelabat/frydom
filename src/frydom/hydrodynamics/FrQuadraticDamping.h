//
// Created by Lucas Letournel on 12/09/18.
//

#ifndef FRYDOM_FRQUADRATICDAMPING_H
#define FRYDOM_FRQUADRATICDAMPING_H
#include "frydom/core/FrForce.h"

namespace frydom {

    class FrQuadraticDamping : public FrForce {

    private:
        double m_Cu = 0;
        double m_Cv = 0;
        double m_Cw = 0;

        double m_Su = 0;
        double m_Sv = 0;
        double m_Sw = 0;

        bool m_relativeVelocity = true;

    public:

        FrQuadraticDamping() {};

        void SetDampingCoefficients(double Cu, double Cv, double Cw) {
            m_Cu = Cu;
            m_Cv = Cv;
            m_Cw = Cw;
        }

        void SetProjectedSections(double Su, double Sv,double Sw) {
            m_Su = Su;
            m_Sv = Sv;
            m_Sw = Sw;
        }

        chrono::ChVector<double> GetProjectedSections(){
            return chrono::ChVector<double>(m_Su,m_Sv,m_Sw);
        }

        chrono::ChVector<double> GetDampingCoefficients(){
            return chrono::ChVector<double>(m_Cu,m_Cv,m_Cw);
        }

        void Initialize() override;

        void SetLogPrefix(std::string prefix_name) override {
            if (prefix_name=="") {
                m_logPrefix = "FquadDamp_" + FrForce::m_logPrefix;
            } else {
                m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
            }
        }

        void UpdateState() override;

    };


};  // end namespace frydom


#endif //FRYDOM_FRQUADRATICDAMPING_H
