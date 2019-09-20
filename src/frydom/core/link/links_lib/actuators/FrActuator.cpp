//
// Created by frongere on 06/02/19.
//

#include "FrActuator.h"

#include "frydom/core/link/links_lib/FrLink.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"


namespace frydom {

    template<typename OffshoreSystemType>
    FrActuator<OffshoreSystemType>::FrActuator(FrLink<OffshoreSystemType> *actuatedLink) :
        FrLinkBase<OffshoreSystemType>(actuatedLink->GetNode1(), actuatedLink->GetNode2(), actuatedLink->GetSystem()),
        m_actuatedLink(actuatedLink) {}

    template<typename OffshoreSystemType>
    bool FrActuator<OffshoreSystemType>::IsDisabled() const {
      return GetChronoItem_ptr()->IsDisabled(); // TODO : voir si on teste aussi m_actuatedLink
    }

    template<typename OffshoreSystemType>
    void FrActuator<OffshoreSystemType>::SetDisabled(bool disabled) {
      GetChronoItem_ptr()->SetDisabled(disabled);
    }

//    bool FrActuator<OffshoreSystemType>::IsBroken() const {
//        return m_chronoMotor->IsBroken();
//    }
//
//    void FrActuator<OffshoreSystemType>::SetBroken(bool broken) {
//
//    }
    template<typename OffshoreSystemType>
    bool FrActuator<OffshoreSystemType>::IsActive() const {
      return true;
    }

    template<typename OffshoreSystemType>
    Force FrActuator<OffshoreSystemType>::GetMotorForceInBody1(FRAME_CONVENTION fc) const {
      auto nodeFrame_WRT_COG = this->m_node1->GetFrameWRT_COG_InBody();
      return nodeFrame_WRT_COG.template ProjectVectorFrameInParen<Force>(GetMotorForceInNode(fc), fc);
    }

    template<typename OffshoreSystemType>
    Force FrActuator<OffshoreSystemType>::GetMotorForceInBody2(FRAME_CONVENTION fc) const {
      auto nodeFrame_WRT_COG = this->m_node2->GetFrameWRT_COG_InBody();
      return -nodeFrame_WRT_COG.template ProjectVectorFrameInParen<Force>(GetMotorForceInNode(fc), fc);
    }

    template<typename OffshoreSystemType>
    Torque FrActuator<OffshoreSystemType>::GetMotorTorqueAtCOGInBody1(FRAME_CONVENTION fc) const {
      auto nodeFrame_WRT_COG = this->m_node1->GetFrameWRT_COG_InBody();

      auto torqueAtNode1_ref = nodeFrame_WRT_COG.template ProjectVectorFrameInParen<Torque>(GetMotorTorqueInNode(fc),
                                                                                            fc);
      auto COG_M1_ref = nodeFrame_WRT_COG.GetPosition(fc);
      auto force_ref = nodeFrame_WRT_COG.template ProjectVectorFrameInParen<Force>(GetMotorForceInNode(fc), fc);

      return torqueAtNode1_ref + COG_M1_ref.cross(force_ref);
    }

    template<typename OffshoreSystemType>
    Torque FrActuator<OffshoreSystemType>::GetMotorTorqueInBody1(FRAME_CONVENTION fc) const {
      auto nodeFrame_WRT_COG = this->m_node1->GetFrameWRT_COG_InBody();
      return nodeFrame_WRT_COG.template ProjectVectorFrameInParen<Torque>(GetMotorTorqueInNode(fc), fc);
    }

    template<typename OffshoreSystemType>
    Torque FrActuator<OffshoreSystemType>::GetMotorTorqueAtCOGInBody2(FRAME_CONVENTION fc) const {
      auto nodeFrame_WRT_COG = this->m_node2->GetFrameWRT_COG_InBody();

      auto torqueAtNode2_ref = nodeFrame_WRT_COG.template ProjectVectorFrameInParen<Torque>(GetMotorTorqueInNode(fc),
                                                                                            fc);
      auto COG_M2_ref = nodeFrame_WRT_COG.GetPosition(fc);
      auto force_ref = nodeFrame_WRT_COG.template ProjectVectorFrameInParen<Force>(GetMotorForceInNode(fc), fc);

      return -(torqueAtNode2_ref + COG_M2_ref.cross(force_ref));
    }

    template<typename OffshoreSystemType>
    Torque FrActuator<OffshoreSystemType>::GetMotorTorqueInBody2(FRAME_CONVENTION fc) const {
      auto nodeFrame_WRT_COG = this->m_node2->GetFrameWRT_COG_InBody();
      return nodeFrame_WRT_COG.template ProjectVectorFrameInParen<Torque>(-GetMotorTorqueInNode(fc), fc);
    }

    template<typename OffshoreSystemType>
    void FrActuator<OffshoreSystemType>::AddFields() {

      this->m_message->template AddField<double>("time", "s", "Current time of the simulation",
                                                 [this]() { return this->m_system->GetTime(); });

      this->m_message->template AddField<double>
          ("MotorPower", "kW", "power delivered by the motor", [this]() { return GetMotorPower(); });
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("MotorForceInBody1", "N",
           fmt::format("Force applied by the motor on body 1, in body 1 reference frame {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetMotorForceInBody1(this->GetLogFrameConvention()); });
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("MotorForceInBody2", "N",
           fmt::format("Force applied by the motor on body 1, in body 2 reference frame {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetMotorForceInBody2(this->GetLogFrameConvention()); });
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("MotorTorqueInBody1(", "N",
           fmt::format("Torque applied by the motor on body 1, in body 1 reference frame {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetMotorTorqueInBody1(this->GetLogFrameConvention()); });
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("MotorTorqueInBody2(", "N",
           fmt::format("Torque applied by the motor on body 2, in body 2 reference frame {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetMotorTorqueInBody2(this->GetLogFrameConvention()); });
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("MotorTorqueAtCOGInBody1(", "N",
           fmt::format("Torque applied by the motor at COG on body 1, in body 1 reference frame {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetMotorTorqueAtCOGInBody1(this->GetLogFrameConvention()); });
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("MotorTorqueAtCOGInBody2(", "N",
           fmt::format("Torque applied by the motor at COG on body 2, in body 2 reference frame {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetMotorTorqueAtCOGInBody2(this->GetLogFrameConvention()); });

    }

}  // end namespace frydom
