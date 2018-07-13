//
// Created by camille on 28/05/18.
//

#ifndef FRYDOM_FRMANOEUVRINGDAMPING_H
#define FRYDOM_FRMANOEUVRINGDAMPING_H

#include "FrForce"

class FrManoeuvringDamping : public FrForce {

private:
    eigen::matrix<6,6> A0;
public:
    /// Default constructor
    FrManoeuvringDamping() {};

    FrManoeuvringDamping(FrHydroDB* HDB, FrOffshoreSystem* system) {



    }

    void SetAddedMassCoeff();

    /// Update force
    void UpdateState() override {

        auto vel = GetBody()->GetCurrentRelativeVelocity();
        auto ang = GetBody()->GetRot_dt();

        auto u = std::vector<double>(vel.x(), vel.y(), vel.z(), ang.x(), ang.y(), ang.z());

        force = 0.;
        for (unsigned int i=0; i<6; i++) {
            force.x() += (A0.at(1,i)*u[5] - A0.at(2,i)*u[4]) * u[i];
            force.y() += (A0.at(2,i)*u[3] - A0.at(0,i)*u[5]) * u[i];
            force.z() += (A0.at(0,i)*u[4] - A0.at(1,i)*u[3]) * u[i];
        }

        moment = 0.;
        for (unsigned int i=0; i<6; i++) {
            moment.x() += (A0.at(1,i)*u[2] - A0.at(2,i)*u[1] + A0.at(4,i)*u[5] - A0.at(5,i)*u[4]) * u[i];
            moment.y() += (A0.at(2,i)*u[0] - A0.at(0,i)*u[2] + A0.at(5,i)*u[3] - A0.at(3,i)*u[5]) * u[i];
            moment.z() += (A0.at(0,i)*u[1] - A0.at(1,i)*u[0] + A0.at(3,i)*u[4] - A0.at(4,i)*u[3]) * u[i];
        }

    }

};


#endif //FRYDOM_FRMANOEUVRINGDAMPING_H
