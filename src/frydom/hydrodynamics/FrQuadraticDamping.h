//
// Created by Lucas Letournel on 12/09/18.
//

#ifndef FRYDOM_FRQUADRATICDAMPING_H
#define FRYDOM_FRQUADRATICDAMPING_H
#include "frydom/core/FrForce.h"

namespace frydom {

    class FrQuadraticDamping : public FrForce {

    private:
        chrono::ChVector<double> m_translationalDampings = chrono::VNULL;
        chrono::ChVector<double> m_rotationalDampings = chrono::VNULL;

        chrono::ChVector<double> m_projectedSection = chrono::VNULL;

    public:

        FrQuadraticDamping() {};

        void SetManeuveuringDampings(const chrono::ChVector<double>& dampings) {
            m_translationalDampings.x() = dampings.x();
            m_translationalDampings.y() = dampings.y();
            m_rotationalDampings.z() = dampings.z();
        }

        void SetManeuveuringDampings(double Bx, double By, double Byaw) {
            SetManeuveuringDampings(chrono::ChVector<double>(Bx, By, Byaw));
        }

        void SetSeakeepingDampings(const chrono::ChVector<double>& dampings) {
            m_translationalDampings.z() = dampings.x();
            m_rotationalDampings.x() = dampings.y();
            m_rotationalDampings.y() = dampings.z();
        }

        void SetSeakeepingDampings(double Bz, double Broll, double Bpitch) {
            SetSeakeepingDampings(chrono::ChVector<double>(Bz, Broll, Bpitch));
        }

        void SetTranslationalDampings(const chrono::ChVector<double>& dampings) {
            m_translationalDampings = dampings;
        }

        void SetTranslationalDampings(double Bx, double By, double Bz) {
            SetTranslationalDampings(chrono::ChVector<double>(Bx, By, Bz));
        }

        void SetRotationalDampings(const chrono::ChVector<double>& dampings) {
            m_rotationalDampings = dampings;
        }

        void SetRotationalDampings(double Broll, double Bpitch, double Byaw) {
            SetRotationalDampings(chrono::ChVector<double>(Broll, Bpitch, Byaw));
        }

        void SetProjectedSection(const chrono::ChVector<double>& section) {
            m_projectedSection = section;
        }

        void SetProjectedSection(double Sx, double Sy, double Sz) {
            SetProjectedSection(chrono::ChVector<double>(Sx,Sy,Sz));
        }

        chrono::ChVector<double> GetProjectedSection() {
            return m_projectedSection;
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
