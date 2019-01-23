//
// Created by frongere on 23/01/19.
//

#ifndef FRYDOM_FRPRISMATICLINK_H
#define FRYDOM_FRPRISMATICLINK_H


//#include "chrono/physics/ChLinkLock.h"

#include "frydom/core/link/FrLink.h"


namespace frydom {


    class FrPrismaticLink;
    std::shared_ptr<FrPrismaticLink> make_prismatic_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);


    class FrPrismaticLink : public _FrLinkLockBase {

    public:
        FrPrismaticLink(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);


        double GetPosition() const;

        double GetVelocity() const;

        double GetAcceleration() const;




        void Initialize() override;

        void StepFinalize() override;


    };




}  // end namespace frydom

#endif //FRYDOM_FRPRISMATICLINK_H
