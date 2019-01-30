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

    void FrPrismaticLink::SetSpringDamper(double stiffness, double dampingCoeff, double restLength) {

    }

    Direction FrPrismaticLink::GetLinkDirectionInWorld() const {
//        m_chronoLink->GetLinkRelativeCoords();
    }

    double FrPrismaticLink::GetLinkPosition() const {
        return -m_chronoLink->GetRelM().pos[2]; // Plutot appeler des methodes de FrLinkLockBase
    }

    double FrPrismaticLink::GetLinkVelocity() const {

    }

    double FrPrismaticLink::GetLinkAcceleration() const {

    }

    void FrPrismaticLink::Initialize() {
        FrLink_::Initialize();
    }

    void FrPrismaticLink::Update(double time) {

        std::cout << m_chronoLink->GetRelativeAcceleration() << std::endl;

        // Position relative entre marqueurs
//        m_chronoLink->GetLinkRelativeCoords()
//        m_node1-
//        m_node1->GetZAxisInWorld();

        // TODO : continuer a tester !!!







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
