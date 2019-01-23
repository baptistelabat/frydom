//
// Created by frongere on 20/09/18.
//

#include <frydom/core/common/FrNode.h>
#include "FrLink.h"


namespace frydom {

    FrLink_::FrLink_(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_ *system) :
            m_node1(node1), m_node2(node2), m_system(system) {}

//    void FrLink_::SetMarkers(FrNode_ *node1, FrNode_ *node2) {
//
//        auto marker1 = node1->m_chronoMarker;
//        auto marker2 = node2->m_chronoMarker;
//
//
//
//
//    }

    FrNode_* FrLink_::GetNode1() {
        return m_node1.get();
    }

    FrNode_* FrLink_::GetNode2() {
        return m_node2.get();
    }







    _FrLinkLockBase::_FrLinkLockBase(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2,
                                     FrOffshoreSystem_ *system) : FrLink_(node1, node2, system) {

    }








//    FrPrismaticLink::FrPrismaticLink(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2,
//            FrOffshoreSystem_ *system) : FrLink_(node1, node2, system) {
//
//        m_chronoLink = std::make_shared<chrono::ChLinkLockPrismatic>();
//
//
//
//    }


//    _FrLinkBase::_FrLinkBase(FrLink_ *link) : m_frydomLink(link) {
//
//    }
//
//    void _FrLinkBase::SetupInitial() {
//        m_frydomLink->Initialize();
//    }
//
//    void _FrLinkBase::Update(bool update_assets) {
//        ChLink::Update(update_assets);
//        m_frydomLink->Update();
//    }
//
//
//    FrLink_::FrLink_() = default;
//
//
//    FrOffshoreSystem_ *FrLink_::GetSystem() {
//        return m_system;
//    }
//
//    void FrLink_::SetName(const char *name) {
//        m_chronoLink->SetName(name);
//    }
//
//    std::string FrLink_::GetName() const {
//        return m_chronoLink->GetNameString();
//    }


//
//
//
//    _FrLinearActuatorBase::_FrLinearActuatorBase(FrLink_ *link) {
//
//    }
//
//    void _FrLinearActuatorBase::SetupInitial() {
//
//    }
//
//    void _FrLinearActuatorBase::Update(bool update_assets) {
//        ChLink::Update(update_assets);
//    }




}  // end namespace frydom