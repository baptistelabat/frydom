// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#include "FrRadiationForce.h"

#include "FrRadiationModel.h"
#include "frydom/core/body/FrBody.h"


namespace frydom {

    // --------------------------------------------------
    // FrRadiationForce
    // --------------------------------------------------

    FrRadiationForce::FrRadiationForce(FrRadiationModel* radiationModel)
            : m_radiationModel(radiationModel) {}

    void FrRadiationForce::SetRadiationModel(FrRadiationModel* radiationModel) {
        m_radiationModel = radiationModel;
    }

    // --------------------------------------------------
    // FrRadiationConvolutionForce
    // --------------------------------------------------

    FrRadiationConvolutionForce::FrRadiationConvolutionForce(
            FrRadiationConvolutionModel* radiationModel)
            : FrRadiationForce(radiationModel) {}

    void FrRadiationConvolutionForce::AddFields() {

//        if (IsLogged()) {
//
//            // Log time
//            m_message->AddField<double>("time", "s", "Current time of the simulation",
//                                        [this]() { return m_chronoForce->GetChTime(); });
//
//            // Log of the convolution part of the force
//            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                    ("ForceConvolutionInBody","N", fmt::format("Convolution part of the force in body reference frame in {}", GetLogFrameConvention()),
//                     [this]() {return GetForceInBody(GetLogFrameConvention());});
//
//            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                    ("TorqueConvolutionInBodyAtCOG","Nm", fmt::format("Convolution part of the torque at COG in body reference frame in {}", GetLogFrameConvention()),
//                     [this]() {return GetTorqueInBodyAtCOG(GetLogFrameConvention());});
//
//            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                    ("ForceConvolutionInWorld","N", fmt::format("Convolution part of the force in world reference frame in {}", GetLogFrameConvention()),
//                     [this]() {return GetForceInWorld(GetLogFrameConvention());});
//
//            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                    ("TorqueConvolutionInWorldAtCOG","Nm", fmt::format("Convolution path of the torque at COG in world reference frame in {}", GetLogFrameConvention()),
//                     [this]() {return GetTorqueInWorldAtCOG(GetLogFrameConvention());});
//
//            // Log the inertia part of the force
//            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                    ("ForceInertiaInBody", "N", fmt::format("Inertia part of the force in body reference frame in {}", GetLogFrameConvention()),
//                     [this]() {return GetForceInertiaPartInBody(GetLogFrameConvention());});
//
//            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                    ("TorqueInertiaInBodyAtCOG", "N.m", fmt::format("Inertia part of the force in body reference frame in {}", GetLogFrameConvention()),
//                     [this]() {return GetTorqueInertiaPartInBody(GetLogFrameConvention());});
//
//            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                    ("ForceInertiaInWorld", "N", fmt::format("Inertia part of the force in world reference frame in {}", GetLogFrameConvention()),
//                     [this]() {return GetForceInertiaPartInWorld(GetLogFrameConvention());});
//
//            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                    ("TorqueInertiaInWorldAtCOG", "Nm", fmt::format("Inertia part of the torque at COG in world reference frame in {}", GetLogFrameConvention()),
//                     [this]() {return GetTorqueInertiaPartInWorld(GetLogFrameConvention());});
//
//            // Log the whole radiation force
//            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                    ("ForceInBody","N", fmt::format("Convolution part of the force in body reference frame in {}", GetLogFrameConvention()),
//                     [this]() {return GetForceInBody(GetLogFrameConvention()) + GetForceInertiaPartInBody(GetLogFrameConvention());});
//
//            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                    ("TorqueInBodyAtCOG","Nm", fmt::format("Convolution part of the torque at COG in body reference frame in {}", GetLogFrameConvention()),
//                     [this]() {return GetTorqueInBodyAtCOG(GetLogFrameConvention()) + GetTorqueInertiaPartInBody(GetLogFrameConvention());});
//
//            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                    ("ForceInWorld","N", fmt::format("Convolution part of the force in world reference frame in {}", GetLogFrameConvention()),
//                     [this]() {return GetForceInWorld(GetLogFrameConvention()) + GetForceInertiaPartInWorld(GetLogFrameConvention());});
//
//            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                    ("TorqueInWorldAtCOG","Nm", fmt::format("Convolution path of the torque at COG in world reference frame in {}", GetLogFrameConvention()),
//                     [this]() {return GetTorqueInWorldAtCOG(GetLogFrameConvention()) + GetTorqueInertiaPartInWorld(GetLogFrameConvention());});
//        }

    }

    void FrRadiationConvolutionForce::Initialize() {
        FrRadiationForce::Initialize();
    }

    void FrRadiationConvolutionForce::StepFinalize() {
        this->UpdateForceInertiaPart();
        FrRadiationForce::StepFinalize();
    }

    void FrRadiationConvolutionForce::Compute(double time) {

        auto force = m_radiationModel->GetRadiationForce(m_body);
        auto torque = m_radiationModel->GetRadiationTorque(m_body);

        SetForceTorqueInWorldAtCOG(force, torque, NWU);

        this->UpdateForceInertiaPart();
    }

    void FrRadiationConvolutionForce::UpdateForceInertiaPart() {

        auto radiationModel = dynamic_cast<FrRadiationConvolutionModel*>(m_radiationModel);
        auto forceInertiaPart = radiationModel->GetRadiationInertiaPart(GetBody());
        c_forceInertiaPart = forceInertiaPart.GetForce();
        c_torqueInertiaPart = forceInertiaPart.GetTorque();
    }

    Force FrRadiationConvolutionForce::GetForceInertiaPartInBody(FRAME_CONVENTION fc) const {
        auto force = c_forceInertiaPart;
        if (IsNED(fc)) { internal::SwapFrameConvention<Force>(force); }
        return force;
    }

    Torque FrRadiationConvolutionForce::GetTorqueInertiaPartInBody(FRAME_CONVENTION fc) const {
        auto torque = c_torqueInertiaPart;
        if (IsNED(fc)) { internal::SwapFrameConvention<Torque>(torque); }
        return torque;
    }

    Force FrRadiationConvolutionForce::GetForceInertiaPartInWorld(FRAME_CONVENTION fc) const {
        auto force = GetBody()->ProjectVectorInWorld<Force>(c_forceInertiaPart, fc);
        return force;
    }

    Torque FrRadiationConvolutionForce::GetTorqueInertiaPartInWorld(FRAME_CONVENTION fc) const {
        auto torque = GetBody()->ProjectVectorInWorld(c_torqueInertiaPart, fc);
        return torque;
    }

}  // end namespace frydom
