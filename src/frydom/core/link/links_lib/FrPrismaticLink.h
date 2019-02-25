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
        double m_restLength = 0.;

        double m_linkPosition = 0.;
        double m_linkVelocity = 0.;
        double m_linkAcceleration = 0.;

    public:
        /// Constructor from two nodes and a pointer to the system.
        /// It automatically adds the link to the system
        FrPrismaticLink(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);

        /// Set the spring and damper coefficients
        void SetSpringDamper(double stiffness, double damping);

        /// Set the rest length of the link
        void SetRestLength(double restLength);

        /// Get the rest length of the link
        double GetRestLength() const;



        /// Get the direction of the link in world woordinate system
        const Direction GetLinkDirectionInWorld(FRAME_CONVENTION fc) const;


        /// Get the link position with respect to the rest length
        double GetLinkPosition() const;

        /// Get the link velocity along the z axis
        double GetLinkVelocity() const;

        /// Get the link acceleration along the z axis
        double GetLinkAcceleration() const;

        /// Get the link force applying on body 2, along the z axis of the link
        double GetLinkForce() const;

        /// Get the power delivered by the force in the link, along the z axis of the link
        double GetLinkPower() const override;

        /// Initialize the link
        void Initialize() override;

        /// Update the link
        void Update(double time) override;

        /// Called after every time step, only once
        void StepFinalize() override;

        /// Compute the link force. Here this is essentially a torque with a default spring damper.
        void UpdateForces(double time);

    private:

        /// Update the link cache
        void UpdateCache() override;

    };


    /// Helper function to make it easy to link two nodes by a prismatic link
    std::shared_ptr<FrPrismaticLink> make_prismatic_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);


}  // end namespace frydom

#endif //FRYDOM_FRPRISMATICLINK_H