//
// Created by frongere on 20/09/18.
//

#include "FrLink.h"

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMotor.h"

#include <frydom/core/common/FrNode.h>





namespace frydom {


    // FrLinkBase_ method definitions

    FrLinkBase_::FrLinkBase_(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_ *system) :
            m_node1(node1), m_node2(node2) {
        m_system = system;
    }

    FrNode_* FrLinkBase_::GetNode1() {
        return m_node1.get();
    }

    FrNode_* FrLinkBase_::GetNode2() {
        return m_node2.get();
    }

    FrBody_* FrLinkBase_::GetBody1() {
        return m_node1->GetBody();
    }

    FrBody_* FrLinkBase_::GetBody2() {
        return m_node2->GetBody();
    }

    FrFrame_ FrLinkBase_::GetTransformFromFrame2ToFrame1() const {
        return m_node2->GetFrameInWorld().GetOtherFrameRelativeTransform_WRT_ThisFrame(m_node1->GetFrameInWorld());
    }






    // FrLink_ method definitions

    FrLink_::FrLink_(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2,
                     FrOffshoreSystem_ *system) : FrLinkBase_(node1, node2, system) {}


    void FrLink_::SetMarkers(FrNode_* node1, FrNode_* node2) {
        m_chronoLink->SetUpMarkers(node1->m_chronoMarker.get(), node2->m_chronoMarker.get());
    }

    std::shared_ptr<chrono::ChLink> FrLink_::GetChronoLink() {
        return m_chronoLink;
    }

    bool FrLink_::IsDisabled() const {
        return m_chronoLink->IsDisabled();
    }

    void FrLink_::SetDisabled(bool disabled) {
        m_chronoLink->SetDisabled(disabled);
    }

    bool FrLink_::IsBroken() const {
        return m_chronoLink->IsBroken();
    }

    void FrLink_::SetBroken(bool broken) {
        m_chronoLink->SetBroken(broken);
    }

    bool FrLink_::IsActive() const {
        return m_chronoLink->IsActive();
    }

    const Force FrLink_::GetLinkReactionForceInLinkFrame1() const { // TODO : tester
        auto transform2To1 = GetTransformFromFrame2ToFrame1();
        auto forceIn2 = GetLinkReactionForceInLinkFrame2();
        return transform2To1.ProjectVectorParentInFrame<Force>(forceIn2, NWU); // TODO : voir si on reporte le fc...
    }

    const Force FrLink_::GetLinkReactionForceInLinkFrame2() const { // TODO : tester
        return internal::ChVectorToVector3d<Force>(m_chronoLink->Get_react_force());
    }

    const Force FrLink_::GetLinkReactionForceInWorldFrame(FRAME_CONVENTION fc) const { // TODO : tester
        return m_node2->ProjectVectorInWorld<Force>(GetLinkReactionForceInLinkFrame2(), fc);
    }

    const Torque FrLink_::GetLinkReactionTorqueInLinkFrame1() const { // TODO : tester
        auto transform2To1 = GetTransformFromFrame2ToFrame1();
        auto torqueIn2 = GetLinkReactionForceInLinkFrame2();
        return transform2To1.ProjectVectorParentInFrame<Force>(torqueIn2, NWU); // TODO : voir si on reporte le fc...
    }

    const Torque FrLink_::GetLinkReactionTorqueInLinkFrame2() const { // TODO : tester
        return internal::ChVectorToVector3d<Torque>(m_chronoLink->Get_react_torque());
    }

    const Torque FrLink_::GetLinkReactionTorqueInWorldFrame(FRAME_CONVENTION fc) const { // TODO : tester
        return m_node2->ProjectVectorInWorld<Torque>(GetLinkReactionForceInLinkFrame2(), fc);
    }





}  // end namespace frydom
