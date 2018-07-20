//
// Created by camille on 28/05/18.
//

#ifndef FRYDOM_FRMANOEUVRINGDAMPING_H
#define FRYDOM_FRMANOEUVRINGDAMPING_H

#include "FrForce"

namespace frydom {

    class FrManoeuvringDamping : public FrForce {


    };


    class FrTaylorManDamping : public FrForce {

        struct TypeCoeff {
            double x = 0.;
            double y = 0.;
            double n = 0.;
        };

    private:
        unsigned int m_order;                           ///< Order of the taylor expansion
        std::vector <std::vector<TypeCoeff>> m_coeff;   ///< Taylor coefficients

    public:
        /// Default constructor
        FrTaylorManDamping() = default;

        /// Set order of the taylor model expansion
        void SetOrder(const unsigned int n);

        /// Return the order of the taylor expansion used
        unsigned int GetOrder() const { return m_order; }

        /// Set taylor coefficients defined by tag
        void Set(const std::string tag, const double val);

        /// Set taylor coefficients for surge motion
        void SetX(const std::string tag, const double val);

        /// Set taylor coefficients for sway motion
        void SetY(const std::string tag, const double val);

        /// Set taylor coefficients for yaw motion
        void SetN(const std::string tag, const double val);

        void UpdateState() override;


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
