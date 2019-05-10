//
// Created by frongere on 06/02/19.
//

#include "FrActuator.h"

#include "frydom/core/link/links_lib/FrLink.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"


namespace frydom {


    FrActuator::FrActuator(FrLink *actuatedLink) :
        FrLinkBase(actuatedLink->GetNode1(), actuatedLink->GetNode2(), actuatedLink->GetSystem()),
        m_actuatedLink(actuatedLink) {}

    bool FrActuator::IsDisabled() const {
        return GetChronoItem_ptr()->IsDisabled(); // TODO : voir si on teste aussi m_actuatedLink
    }

    void FrActuator::SetDisabled(bool disabled) {
        GetChronoItem_ptr()->SetDisabled(disabled);
    }

//    bool FrActuator::IsBroken() const {
//        return m_chronoMotor->IsBroken();
//    }
//
//    void FrActuator::SetBroken(bool broken) {
//
//    }

    bool FrActuator::IsActive() const {
        return false;
    }

    Force FrActuator::GetMotorForceInBody1(FRAME_CONVENTION fc) const {
        auto markerFrame_WRT_COG = m_node1->GetFrameWRT_COG_InBody();
        return markerFrame_WRT_COG.ProjectVectorFrameInParent<Force>(GetMotorForceInMarker(fc), fc);
    }

    Force FrActuator::GetMotorForceInBody2(FRAME_CONVENTION fc) const {
        auto markerFrame_WRT_COG = m_node2->GetFrameWRT_COG_InBody();
        return -markerFrame_WRT_COG.ProjectVectorFrameInParent<Force>(GetMotorForceInMarker(fc), fc);
    }


    Torque FrActuator::GetMotorTorqueAtCOGInBody1(FRAME_CONVENTION fc) const {
        auto markerFrame_WRT_COG = m_node1->GetFrameWRT_COG_InBody();

        auto torqueAtMarker1_ref = markerFrame_WRT_COG.ProjectVectorFrameInParent<Torque>(GetMotorTorqueInMarker(fc), fc);
        auto COG_M1_ref = markerFrame_WRT_COG.GetPosition(fc);
        auto force_ref = markerFrame_WRT_COG.ProjectVectorFrameInParent<Force>(GetMotorForceInMarker(fc), fc);

        return torqueAtMarker1_ref + COG_M1_ref.cross(force_ref);
    }

    Torque FrActuator::GetMotorTorqueAtCOGInBody2(FRAME_CONVENTION fc) const {
        auto markerFrame_WRT_COG = m_node2->GetFrameWRT_COG_InBody();

        auto torqueAtMarker2_ref = markerFrame_WRT_COG.ProjectVectorFrameInParent<Torque>(GetMotorTorqueInMarker(fc), fc);
        auto COG_M2_ref = markerFrame_WRT_COG.GetPosition(fc);
        auto force_ref = markerFrame_WRT_COG.ProjectVectorFrameInParent<Force>(GetMotorForceInMarker(fc), fc);

        return -(torqueAtMarker2_ref + COG_M2_ref.cross(force_ref));
    }


}  // end namespace frydom
