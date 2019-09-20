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
    template<typename OffshoreSystemType>
    FrRadiationForce<OffshoreSystemType>::FrRadiationForce(FrRadiationModel<OffshoreSystemType> *radiationModel)
        : m_radiationModel(radiationModel) {}

    template<typename OffshoreSystemType>
    void FrRadiationForce<OffshoreSystemType>::SetRadiationModel(FrRadiationModel<OffshoreSystemType> *radiationModel) {
      m_radiationModel = radiationModel;
    }

    // --------------------------------------------------
    // FrRadiationConvolutionForce
    // --------------------------------------------------
    template<typename OffshoreSystemType>
    FrRadiationConvolutionForce<OffshoreSystemType>::FrRadiationConvolutionForce(
        FrRadiationConvolutionModel<OffshoreSystemType> *radiationModel)
        : FrRadiationForce<OffshoreSystemType>(radiationModel) {}

    template<typename OffshoreSystemType>
    void FrRadiationConvolutionForce<OffshoreSystemType>::AddFields() {

      if (this->IsLogged()) {

        // Log time
        this->m_message->template AddField<double>("time", "s", "Current time of the simulation",
                                                   [this]() { return this->m_chronoForce->GetChTime(); });

        // Log of the convolution part of the force
        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("ForceConvolutionInBody", "N",
             fmt::format("Convolution part of the force in body reference frame in {}", this->GetLogFrameConvention()),
             [this]() { return GetForceInBody(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("TorqueConvolutionInBodyAtCOG", "Nm",
             fmt::format("Convolution part of the torque at COG in body reference frame in {}",
                         this->GetLogFrameConvention()),
             [this]() { return GetTorqueInBodyAtCOG(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("ForceConvolutionInWorld", "N",
             fmt::format("Convolution part of the force in world reference frame in {}", this->GetLogFrameConvention()),
             [this]() { return GetForceInWorld(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("TorqueConvolutionInWorldAtCOG", "Nm",
             fmt::format("Convolution path of the torque at COG in world reference frame in {}",
                         this->GetLogFrameConvention()),
             [this]() { return GetTorqueInWorldAtCOG(this->GetLogFrameConvention()); });

        // Log the inertia part of the force
        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("ForceInertiaInBody", "N",
             fmt::format("Inertia part of the force in body reference frame in {}", this->GetLogFrameConvention()),
             [this]() { return GetForceInertiaPartInBody(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("TorqueInertiaInBodyAtCOG", "N.m",
             fmt::format("Inertia part of the force in body reference frame in {}", this->GetLogFrameConvention()),
             [this]() { return GetTorqueInertiaPartInBody(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("ForceInertiaInWorld", "N",
             fmt::format("Inertia part of the force in world reference frame in {}", this->GetLogFrameConvention()),
             [this]() { return GetForceInertiaPartInWorld(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("TorqueInertiaInWorldAtCOG", "Nm",
             fmt::format("Inertia part of the torque at COG in world reference frame in {}",
                         this->GetLogFrameConvention()),
             [this]() { return GetTorqueInertiaPartInWorld(this->GetLogFrameConvention()); });

        // Log the whole radiation force
        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("ForceInBody", "N",
             fmt::format("Convolution part of the force in body reference frame in {}", this->GetLogFrameConvention()),
             [this]() {
               return GetForceInBody(this->GetLogFrameConvention()) +
                      GetForceInertiaPartInBody(this->GetLogFrameConvention());
             });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("TorqueInBodyAtCOG", "Nm",
             fmt::format("Convolution part of the torque at COG in body reference frame in {}",
                         this->GetLogFrameConvention()),
             [this]() {
               return GetTorqueInBodyAtCOG(this->GetLogFrameConvention()) +
                      GetTorqueInertiaPartInBody(this->GetLogFrameConvention());
             });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("ForceInWorld", "N",
             fmt::format("Convolution part of the force in world reference frame in {}", this->GetLogFrameConvention()),
             [this]() {
               return GetForceInWorld(this->GetLogFrameConvention()) +
                      GetForceInertiaPartInWorld(this->GetLogFrameConvention());
             });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("TorqueInWorldAtCOG", "Nm",
             fmt::format("Convolution path of the torque at COG in world reference frame in {}",
                         this->GetLogFrameConvention()),
             [this]() {
               return GetTorqueInWorldAtCOG(this->GetLogFrameConvention()) +
                      GetTorqueInertiaPartInWorld(this->GetLogFrameConvention());
             });
      }

    }

    template<typename OffshoreSystemType>
    void FrRadiationConvolutionForce<OffshoreSystemType>::Initialize() {
      FrRadiationForce<OffshoreSystemType>::Initialize();
    }

    template<typename OffshoreSystemType>
    void FrRadiationConvolutionForce<OffshoreSystemType>::StepFinalize() {
      this->UpdateForceInertiaPart();
      FrRadiationForce<OffshoreSystemType>::StepFinalize();
    }

    template<typename OffshoreSystemType>
    void FrRadiationConvolutionForce<OffshoreSystemType>::Compute(double time) {

      auto force = this->m_radiationModel->GetRadiationForce(this->m_body);
      auto torque = this->m_radiationModel->GetRadiationTorque(this->m_body);

      SetForceTorqueInWorldAtCOG(force, torque, NWU);

      this->UpdateForceInertiaPart();
    }

    template<typename OffshoreSystemType>
    void FrRadiationConvolutionForce<OffshoreSystemType>::UpdateForceInertiaPart() {

      auto radiationModel = dynamic_cast<FrRadiationConvolutionModel<OffshoreSystemType> *>(this->m_radiationModel);
      auto forceInertiaPart = radiationModel->GetRadiationInertiaPart(this->GetBody());
      c_forceInertiaPart = forceInertiaPart.GetForce();
      c_torqueInertiaPart = forceInertiaPart.GetTorque();
    }

    template<typename OffshoreSystemType>
    Force FrRadiationConvolutionForce<OffshoreSystemType>::GetForceInertiaPartInBody(FRAME_CONVENTION fc) const {
      auto force = c_forceInertiaPart;
      if (IsNED(fc)) { internal::SwapFrameConvention<Force>(force); }
      return force;
    }

    template<typename OffshoreSystemType>
    Torque FrRadiationConvolutionForce<OffshoreSystemType>::GetTorqueInertiaPartInBody(FRAME_CONVENTION fc) const {
      auto torque = c_torqueInertiaPart;
      if (IsNED(fc)) { internal::SwapFrameConvention<Torque>(torque); }
      return torque;
    }

    template<typename OffshoreSystemType>
    Force FrRadiationConvolutionForce<OffshoreSystemType>::GetForceInertiaPartInWorld(FRAME_CONVENTION fc) const {
      auto force = this->GetBody()->template ProjectVectorInWorld<Force>(c_forceInertiaPart, fc);
      return force;
    }

    template<typename OffshoreSystemType>
    Torque FrRadiationConvolutionForce<OffshoreSystemType>::GetTorqueInertiaPartInWorld(FRAME_CONVENTION fc) const {
      auto torque = this->GetBody()->ProjectVectorInWorld(c_torqueInertiaPart, fc);
      return torque;
    }

}  // end namespace frydom
