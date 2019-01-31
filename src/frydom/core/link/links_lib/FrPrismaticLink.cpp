//
// Created by frongere on 23/01/19.
//
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
        m_frame2WRT1_reference.SetZ(restLength, NWU); // TODO : voir si on parametre le FRAME_CONVENTION...
    }

    double FrPrismaticLink::GetRestLength() const {
        return m_frame2WRT1_reference.GetZ(NWU); // TODO : voir si on parametre le FRAME_CONVENTION...
    }

    const Direction FrPrismaticLink::GetLinkDirectionInWorld(FRAME_CONVENTION fc) const {
        return GetNode1()->GetFrameInWorld().GetZAxisInParent(fc);
    }

    double FrPrismaticLink::GetLinkPosition() const {
        return GetMarker2PositionWRTMarker1(NWU).GetZ() - GetRestLength(); // TODO : voir si on parametre le FRAME_CONVENTION...
    }

    double FrPrismaticLink::GetLinkVelocity() const {
        return GetVelocityOfMarker2WRTMarker1(NWU).GetVz(); // TODO : voir si on parametre le FRAME_CONVENTION...
    }

    double FrPrismaticLink::GetLinkAcceleration() const {
        return GetAccelerationOfMarker2WRTMarker1(NWU).GetAccZ(); // TODO : voir si on parametre le FRAME_CONVENTION...
    }

    void FrPrismaticLink::Initialize() {
        FrLink_::Initialize();
    }

    void FrPrismaticLink::Update(double time) {
        FrLink_::Update(time); // It is mandatory to invoke this before all update operations from frydom

        // ICI on appelle de quoi calculer la force dans la liaison





        std::cout << GetLinkPower() << std::endl;

        // Position relative entre marqueurs
//        m_chronoLink->GetLinkRelativeCoords()
//        m_node1-
//        m_node1->GetZAxisInWorld();

        // TODO : continuer a tester !!!
        Force force;
        force.GetFz() = -m_stiffness * GetLinkPosition() - m_damping * GetLinkVelocity();

        SetLinkForceAtMarker1(force);





    }

    void FrPrismaticLink::StepFinalize() {

    }


    std::shared_ptr<FrPrismaticLink>
    make_prismatic_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_ *system) {
        auto link = std::make_shared<FrPrismaticLink>(node1, node2, system);
        system->AddLink(link);


        return link;
    }


}  // end namespace frydom
