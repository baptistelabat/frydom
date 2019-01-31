//
// Created by frongere on 23/01/19.
//

#ifndef FRYDOM_FRPRISMATICLINK_H
#define FRYDOM_FRPRISMATICLINK_H


#include "FrLink.h"


namespace frydom {



    class FrPrismaticLink : public FrLink_ {

    private:
        double m_stiffness = 0.;
        double m_damping = 0.;

    public:
        FrPrismaticLink(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);

        void SetSpringDamper(double stiffness, double damping);

        void SetRestLength(double restLength);

        double GetRestLength() const;



        const Direction GetLinkDirectionInWorld(FRAME_CONVENTION fc) const;


        double GetLinkPosition() const;

        double GetLinkVelocity() const;

        double GetLinkAcceleration() const;



        void Initialize() override;

        void Update(double time) override;

        void StepFinalize() override;


    };


//    class FrPrismaticLink;
    std::shared_ptr<FrPrismaticLink> make_prismatic_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);


}  // end namespace frydom

#endif //FRYDOM_FRPRISMATICLINK_H
