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


#ifndef FRYDOM_FRPRISMATICLINK_H
#define FRYDOM_FRPRISMATICLINK_H


#include "FrLink.h"


namespace frydom {


    /**
     * \class FrPrismaticLink
     * \brief Class for defining a prismatic link.
     */
    class FrPrismaticLink : public FrLink_ {

    private:
        double m_stiffness = 0.; ///> Link linear stiffness
        double m_damping = 0.;   ///> Link linear damping

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
