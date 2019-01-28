//
// Created by frongere on 23/01/19.
//
#include "FrPrismaticLink.h"

//#include "frydom/core/common/FrNode.h"

#include <chrono/physics/ChLinkLock.h>


namespace frydom {


    FrPrismaticLink::FrPrismaticLink(std::shared_ptr<frydom::FrNode_> node1, std::shared_ptr<frydom::FrNode_> node2,
                                     frydom::FrOffshoreSystem_ *system) : FrLink_(node1, node2, system) {
        m_chronoLink = std::make_shared<chrono::ChLinkLockPrismatic>();

        SetMarkers(node1.get(), node2.get());


    }

    void FrPrismaticLink::SetSpringDamper(double stiffness, double dampingCoeff) {

    }

    Direction FrPrismaticLink::GetLinkDirectionInWorld() const {

    }

    double FrPrismaticLink::GetLinkPosition() const {

    }

    double FrPrismaticLink::GetLinkVelocity() const {

    }

    double FrPrismaticLink::GetLinkAcceleration() const {

    }

    void FrPrismaticLink::Initialize() {

    }

    void FrPrismaticLink::Update(double time) {

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
