// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================

#include "FrPrismaticLink.h"

#include "frydom/core/common/FrNode.h"

//#include <chrono/physics/ChLinkLock.h>


namespace frydom {

    FrPrismaticLink::FrPrismaticLink(std::shared_ptr<frydom::FrNode_> node1, std::shared_ptr<frydom::FrNode_> node2,
                                     frydom::FrOffshoreSystem_ *system) : FrLink_(node1, node2, system) {
        m_chronoLink->SetLinkType(PRISMATIC);
    }

    void FrPrismaticLink::SetSpringDamper(double stiffness, double damping) {
        m_stiffness = stiffness;
        m_damping = damping;
    }

    void FrPrismaticLink::SetRestLength(double restLength) {
        m_frame2WRT1_reference.SetZ(restLength, NWU);
        UpdateCache();
    }

    double FrPrismaticLink::GetRestLength() const {
        return m_restLength;
    }

    const Direction FrPrismaticLink::GetLinkDirectionInWorld(FRAME_CONVENTION fc) const {
        return GetNode1()->GetFrameInWorld().GetZAxisInParent(fc);
    }

    double FrPrismaticLink::GetLinkPosition() const {
        return m_linkPosition - m_restLength;
    }

    double FrPrismaticLink::GetLinkVelocity() const {
        return m_linkVelocity;
    }

    double FrPrismaticLink::GetLinkAcceleration() const {
        return m_linkAcceleration;
    }

    double FrPrismaticLink::GetLinkForce() const {
        return GetLinkForceOnBody2InFrame2AtOrigin2(NWU).GetFz();
    }

    double FrPrismaticLink::GetLinkPower() const {
        return GetLinkVelocity() * GetLinkForce();
    }

    void FrPrismaticLink::Initialize() {
        FrLink_::Initialize();

        // Initializing the motor part
        // TODO

        // Initializing the logs
    }

    void FrPrismaticLink::Update(double time) {
        FrLink_::Update(time); // It is mandatory to invoke this before all update operations from frydom

        // Update total link measure
        m_linkPosition = GetMarker2PositionWRTMarker1(NWU).GetZ();
        m_linkVelocity = GetVelocityOfMarker2WRTMarker1(NWU).GetVz();
        m_linkAcceleration = GetAccelerationOfMarker2WRTMarker1(NWU).GetAccZ();


        UpdateForces(time);

    }

    void FrPrismaticLink::StepFinalize() {

    }

    void FrPrismaticLink::UpdateForces(double time) {
        Force force;
        Torque torque;
        force.GetFz() = - m_stiffness * GetLinkPosition() - m_damping * GetLinkVelocity();

        // Using force model from motor

        SetLinkForceTorqueOnBody2InFrame2AtOrigin2(force, torque);
    }

    void FrPrismaticLink::UpdateCache() {
        m_restLength = m_frame2WRT1_reference.GetZ(NWU);
        // FIXME : attention si la liaison n'est pas resolue !!! Ca ne fonctionne pas
    }

    std::shared_ptr<FrPrismaticLink>
    make_prismatic_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_ *system) {
        auto link = std::make_shared<FrPrismaticLink>(node1, node2, system);
        system->AddLink(link);


        return link;
    }


}  // end namespace frydom
