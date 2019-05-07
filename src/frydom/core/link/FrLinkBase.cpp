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


#include "FrLinkBase.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/core/common/FrNode.h"



namespace frydom {

    FrLinkBase::FrLinkBase(std::shared_ptr<FrNode> node1, std::shared_ptr<FrNode> node2, FrOffshoreSystem *system) :
            m_node1(node1), m_node2(node2), m_system(system) {
    }

    std::shared_ptr<FrNode> FrLinkBase::GetNode1() {
        return m_node1;
    }

    const std::shared_ptr<FrNode> FrLinkBase::GetNode1() const {
        return m_node1;
    }

    std::shared_ptr<FrNode> FrLinkBase::GetNode2() {
        return m_node2;
    }

    const std::shared_ptr<FrNode> FrLinkBase::GetNode2() const {
        return m_node2;
    }

    FrBody* FrLinkBase::GetBody1() {
        return m_node1->GetBody();
    }

    FrBody* FrLinkBase::GetBody2() {
        return m_node2->GetBody();
    }

    std::shared_ptr<chrono::ChBody> FrLinkBase::GetChronoBody1() {
        return GetBody1()->GetChronoBody();
    }

    std::shared_ptr<chrono::ChBody> FrLinkBase::GetChronoBody2() {
        return GetBody2()->GetChronoBody();
    }

    void FrLinkBase::InitializeLog(const std::string& rootPath) {

        if (IsLogged()) {

            // Build the path to the link log
            std::string linkPath = fmt::format("{}/{}_{}_{}", rootPath, GetTypeName(), GetName(), GetShortenUUID());
            auto logPath = m_system->GetPathManager()->BuildPath(linkPath, fmt::format("{}_{}.csv",GetTypeName(),GetShortenUUID()));

            // Add the fields to be logged
            // TODO: A completer
            m_message->AddField<double>("time", "s", "Current time of the simulation",
                                        [this]() { return m_system->GetTime(); });

            // Initialize the message
            FrObject::InitializeLog(logPath);
        }

    }

    FrOffshoreSystem *FrLinkBase::GetSystem() {
        return m_system;
    }
//
//    FrFrame FrLinkBase::GetTransformFromFrame2ToFrame1() const {
//        return m_node2->GetFrameInWorld().GetOtherFrameRelativeTransform_WRT_ThisFrame(m_node1->GetFrameInWorld());
//    }


}  // end namespace frydom
