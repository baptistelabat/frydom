//
// Created by frongere on 17/10/17.
//

#ifndef FRYDOM_FRRADIATIONFORCE_H
#define FRYDOM_FRRADIATIONFORCE_H

#include "chrono/physics/ChBody.h"
#include <frydom/core/FrForce.h>
#include <frydom/utils/FrRecorder.h>
#include "FrHydroDB.h"

namespace frydom {


    class FrRadiationForce : public FrForce {

    public:
        FrRadiationForce() = default;

    };


    class FrRadiationConvolutionForce : public FrRadiationForce {
    private:
        FrRecorder<chrono::ChVector<double>> m_linearVelocityRecorder;
        FrRecorder<chrono::ChVector<double>> m_angularVelocityRecorder;
//        FrRadiationIRFDB m_IRFDB;

    public:

//        FrRadiationConvolutionForce() = default;

//        FrRadiationConvolutionForce(FrRadiationIRFDB DB) : m_IRFDB(DB) {}
//
//        void SetRadiationIRFDB(FrRadiationIRFDB DB) {
//            m_IRFDB = DB;
//        }


        void UpdateState() override {

            // 1- Recording the current state
            m_linearVelocityRecorder.record(ChTime, Body->GetPos_dt());
            m_angularVelocityRecorder.record(ChTime, Body->GetWvel_par());

            // 2- Getting the time history for velocities and interpolating it based on impulse response functions
            // discretization



        }



    };





}  // end namespace frydom

#endif //FRYDOM_FRRADIATIONFORCE_H
