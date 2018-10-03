//
// Created by camille on 28/05/18.
//

#ifndef FRYDOM_FRMANOEUVRINGDAMPING_H
#define FRYDOM_FRMANOEUVRINGDAMPING_H

#include "frydom/core/FrForce.h"

namespace frydom {

    class FrManoeuvringDamping : public FrForce {


    };


    class FrTaylorManDamping : public FrForce {

        struct TypeCoeff {
            double val = 0.;
            int cm = 0;
            int m1 = 0;
            int m2 = 0;
            int cn = 0;
            int n1 = 0;
            int n2 = 0;
            int cp = 0;
            int p1 = 0;
            int p2 = 0;
        };

    private:
        std::vector<TypeCoeff> m_cx;                    ///< Taylor coefficients for surge motion
        std::vector<TypeCoeff> m_cy;                    ///< Taylor coefficients for sway motion
        std::vector<TypeCoeff> m_cn;                    ///< Taylor coefficients for yaw motion

        TypeCoeff SetCoeff(const double val, const int m, const int n, const int p);
        TypeCoeff SetCoeff(const std::string tag, const double val);
        double ForceComponent(const TypeCoeff coeff, const double vx, const double vy, const double vrz) const;

    public:
        /// Default constructor
        FrTaylorManDamping() = default;

        /// Set taylor coefficients defined by tag
        void Set(const std::string tag, const double val);

        /// Set taylor coefficients for surge motion
        void SetX(const std::string tag, const double val);

        /// Set taylor coefficients for sway motion
        void SetY(const std::string tag, const double val);

        /// Set taylor coefficients for yaw motion
        void SetN(const std::string tag, const double val);

        void SetX(const double val, const int m, const int n, const int p);

        void SetY(const double val, const int m, const int n, const int p);

        void SetN(const double val, const int m, const int n, const int p);

        void UpdateState() override;

        void SetLogPrefix(std::string prefix_name) override {
            if (prefix_name=="") {
                m_logPrefix = "Fd_man_" + FrForce::m_logPrefix;
            } else {
                m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
            }
        }

    };


    /*
    class FrAddedMassManDamping : public FrForce {


    private:
        eigen::matrix<6, 6> A0;
    public:
        /// Default constructor
        FrManoeuvringDamping() {};

        //FrManoeuvringDamping(FrHydroDB* HDB, FrOffshoreSystem *system) {
        //
        //
        //}

        //void SetAddedMassCoeff();

        /// Update force
        void UpdateState() override {

            auto vel = GetBody()->GetCurrentRelativeVelocity();
            auto ang = GetBody()->GetRot_dt();

            auto u = std::vector<double>(vel.x(), vel.y(), vel.z(), ang.x(), ang.y(), ang.z());

            force = 0.;
            for (unsigned int i = 0; i < 6; i++) {
                force.x() += (A0.at(1, i) * u[5] - A0.at(2, i) * u[4]) * u[i];
                force.y() += (A0.at(2, i) * u[3] - A0.at(0, i) * u[5]) * u[i];
                force.z() += (A0.at(0, i) * u[4] - A0.at(1, i) * u[3]) * u[i];
            }

            moment = 0.;
            for (unsigned int i = 0; i < 6; i++) {
                moment.x() +=
                        (A0.at(1, i) * u[2] - A0.at(2, i) * u[1] + A0.at(4, i) * u[5] - A0.at(5, i) * u[4]) * u[i];
                moment.y() +=
                        (A0.at(2, i) * u[0] - A0.at(0, i) * u[2] + A0.at(5, i) * u[3] - A0.at(3, i) * u[5]) * u[i];
                moment.z() +=
                        (A0.at(0, i) * u[1] - A0.at(1, i) * u[0] + A0.at(3, i) * u[4] - A0.at(4, i) * u[3]) * u[i];
            }

        }

    };
     */
}

#endif //FRYDOM_FRMANOEUVRINGDAMPING_H
