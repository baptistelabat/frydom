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

#ifndef FRYDOM_FRLINK_H
#define FRYDOM_FRLINK_H


#include "frydom/core/link/FrLinkBase.h"

#include <memory>


// Forward declarations
namespace chrono {
    class ChLinkLock;
    class ChLinkMotor;
}


namespace frydom {

    /// Different types of link implemented in FRyDoM
    enum LINK_TYPE {
        CYLINDRICAL,
        FIXED_LINK,
        FREE_LINK,
        PRISMATIC,
        REVOLUTE,
        SPHERICAL,
        PRISMATICREVOLUTE,
        CUSTOM
//        PERPENDICULAR,
//        PARALLEL,
//        PLANEONPLANE,
//        POINTONLINE,
//        POINTONPLANE,
//        SCREW,
//        POINTONSPLINE,
//        DISTANCETOAXIS,
    };


    // Forward declaration
    class FrLink;
    class FrDOFMask;

    namespace internal {

        /**
         * \class FrLinkLockBase
         * \brief Class for defining a link, derived from ChLinkLock.
         */
        struct FrLinkLockBase : public chrono::ChLinkLock {

            using ChronoLinkType = chrono::ChLinkLock::LinkType;

            FrLink* m_frydomLink; ///> Pointer to frydom link object container

            /*
             * Cache data for performance
             */
            FrFrame c_frame1WRT2; ///> the current relative frame of node 1 WRT node 2
            FrFrame c_frame2WRT1; ///> the current relative frame of node 2 WRT node 1

            GeneralizedVelocity c_generalizedVelocity1WRT2;
            GeneralizedVelocity c_generalizedVelocity2WRT1;

            GeneralizedAcceleration c_generalizedAcceleration1WRT2;
            GeneralizedAcceleration c_generalizedAcceleration2WRT1;

            GeneralizedForce c_generalizedForceOnNode2;
            GeneralizedForce c_generalizedForceOnNode1;


            /// Constructor
            explicit FrLinkLockBase(FrLink* frydomLink);

            /// Set the link type based on LINK_TYPE enum
            void SetLinkType(LINK_TYPE lt);

            /// Initialize the link (calls the Initialize method of FrLink
            void SetupInitial() override;

            /// Update (calls the ChLinkLock Update method then the FrLinkUpdate method
            void Update(double time, bool update_assets) override;

            /// Generates the cache variables to speedup further requests on inner link state (frame, velocity, forces...)
            void GenerateCache();

            /// Set a mask. Mainly used in the case of body constraints with the world where we use a FrDOFMaskLink
            void SetMask(FrDOFMask* mask);

            /// Set the link force applying on Body 1 (as external) expressed in node 2 frame and on node 1 origin
            void SetLinkForceOnBody1InFrame2AtOrigin1(const Force &force);

            /// Set the link torque applying on Body 1 (as external) expressed in node 2 frame and on node 1 origin
            void SetLinkTorqueOnBody1InFrame2AtOrigin1(const Torque& torque);

            /// Get the link force applying on Body 1 (as external) expressed in node 2 frame and on node 1 origin
            Force GetLinkForceOnBody1InFrame2AtOrigin1();

            /// Get the link torque applying on Body 1 (as external) expressed in node 2 frame and on node 1 origin
            Torque GetLinkTorqueOnBody1InFrame2ArOrigin1();

            FrFrame GetConstraintViolation();

            void BuildLinkType(chrono::ChLinkLock::LinkType link_type) override;

        };

    }  // end namespace frydom::internal



    /*
     * FrLink
     */

    // Forward declaration
    class FrNode;
    class FrOffshoreSystem;
    class FrActuator;

    /**
     * \class FrLink
     * \brief Class to deal with links, derived from FrLinkBase, instantiate a FrLinkLockBase. Children of FrLink are
     * to set the linkType LINK_TYPE for the instance of the FrLinkLockBase.
     * For one DOF links (REVOLUTE, PRISMATIC ,etc.), a motorisation of the DOF is possible.
     */
    class FrLink : public FrLinkBase {

    protected:

        std::shared_ptr<internal::FrLinkLockBase> m_chronoLink;

        FrFrame m_frame2WRT1_reference;
        bool m_breakable = false; // TODO : utiliser et permettre de declencher la cassure de la liaison

        // Actuator
        std::shared_ptr<FrActuator> m_actuator;

    public:

        /// Constructor taking the nodes attached to the two bodies implied in the link and the system
        FrLink(const std::shared_ptr<FrNode>& node1, const std::shared_ptr<FrNode>& node2, FrOffshoreSystem *system);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "Link"; }

        /// Tells if all constraints of this link are currently turned on or off by the user.
        bool IsDisabled() const override;

        /// User can use this to enable/disable all the constraint of the link as desired.
        void SetDisabled(bool disabled) override;
        /// Make the link breakable
        void SetBreakable(bool breakable);

        /// Is this link breakable ?
        bool IsBreakable() const;

        /// Tells if the link is broken, for excess of pulling/pushing. // TODO : implementer de quoi 'casser' les liaisons si trop d'effort -> foncteurs ?
        bool IsBroken() const;

        /// Set the 'broken' status vof this link.
        void SetBroken(bool broken);

        /// Tells if the link is currently active, in general,
        /// that is tells if it must be included into the system solver or not.
        /// This method cumulates the effect of various flags (so a link may
        /// be not active either because disabled, or broken, or not valid)
        bool IsActive() const override;
        /// Does this link is motorized ?
        bool IsMotorized() const;

        /// Freezes the current configuration as being the reference configuration from which the generalized
        /// position measurement are given (ie. length from the rest length for a prismatic link)
        virtual void SetThisConfigurationAsReference();


        /*
         * Position related methods
         */

        /// Get the Marker 2 frame relatively to node 1 frame
        const FrFrame GetNode2FrameWRTNode1Frame() const;

        /// Get the Marker 1 frame relatively to node 2 frame
        const FrFrame GetNode1FrameWRTNode2Frame() const;

        /// Get the Marker 2 position relatively to node 1, expressed in node 1 frame
        const Position GetNode2PositionWRTNode1(FRAME_CONVENTION fc) const;

        /// Get the Marker 1 position relatively to node 2, expressed in node 2 frame
        const Position GetNode1PositionWRTNode2(FRAME_CONVENTION fc) const;

        /// Get the Marker 2 orientation relatively to node 1
        const FrRotation GetNode2OrientationWRTNode1() const;

        /// Get the Marker 1 orientation relatively to node 2
        const FrRotation GetNode1OrientationWRTNode2() const;

        /*
         * Velocity related methods
         */

        /// Get the Marker 2 generalized velocity with respect to node 1, expressed in node 1 frame
        const GeneralizedVelocity GetGeneralizedVelocityOfNode2WRTNode1(FRAME_CONVENTION fc) const;

        /// Get the Marker 1 generalized velocity with respect to node 2, expressed in node 2 frame
        const GeneralizedVelocity GetGeneralizedVelocityOfNode1WRTNode2(FRAME_CONVENTION fc) const;

        /// Get the Marker 2 velocity with respect to node 1, expressed in node 1 frame
        const Velocity GetVelocityOfNode2WRTNode1(FRAME_CONVENTION fc) const;

        /// Get the Marker 1 velocity with respect to node 2, expressed in node 2 frame
        const Velocity GetVelocityOfNode1WRTNode2(FRAME_CONVENTION fc) const;

        /// Get the Marker 2 angular velocity with respect to node 1, expressed in node 1 frame
        const AngularVelocity GetAngularVelocityOfNode2WRTNode1(FRAME_CONVENTION fc) const;

        /// Get the Marker 1 angular velocity with respect to node 2, expressed in node 2 frame
        const AngularVelocity GetAngularVelocityOfNode1WRTNode2(FRAME_CONVENTION fc) const;

        /*
         * Acceleration related methods
         */

        /// Get the Marker 2 generalized acceleration with respect to node 1, expressed in node 1 frame
        const GeneralizedAcceleration GetGeneralizedAccelerationOfNode2WRTNode1(FRAME_CONVENTION fc) const;

        /// Get the Marker 1 generalized acceleration with respect to node 2, expressed in node 2 frame
        const GeneralizedAcceleration GetGeneralizedAccelerationOfNode1WRTNode2(FRAME_CONVENTION fc) const;

        /// Get the Marker 2 acceleration with respect to node 1, expressed in node 1 frame
        const Acceleration GetAccelerationOfNode1WRTNode2(FRAME_CONVENTION fc) const;

        /// Get the Marker 1 acceleration with respect to node 2, expressed in node 2 frame
        const Acceleration GetAccelerationOfNode2WRTNode1(FRAME_CONVENTION fc) const;

        /// Get the Marker 2 angular acceleration with respect to node 1, expressed in node 1 frame
        const AngularAcceleration GetAngularAccelerationOfNode2WRTNode1(FRAME_CONVENTION fc) const;

        /// Get the Marker 1 angular acceleration with respect to node 2, expressed in node 2 frame
        const AngularAcceleration GetAngularAccelerationOfNode1WRTNode2(FRAME_CONVENTION fc) const;

        /*
         * Force related methods
         */

        /// Get the link reaction force applied at node 1, expressed in node 1 frame.
        /// Note this is the force applied on the body
        const Force GetLinkReactionForceOnNode1(FRAME_CONVENTION fc) const;

        /// Get the link reaction force applied at node 2, expressed in node 2 frame
        /// Note this is the force applied on the body
        const Force GetLinkReactionForceOnNode2(FRAME_CONVENTION fc) const;

        /// Get the link reaction force applied at node 1, expressed in body 1 frame.
        /// Note this is the force applied on the body
        const Force GetLinkReactionForceOnBody1(FRAME_CONVENTION fc) const;

        /// Get the link reaction force applied at node 2, expressed in body 2 frame
        /// Note this is the force applied on the body
        const Force GetLinkReactionForceOnBody2(FRAME_CONVENTION fc) const;

        /// Get the link reaction torque applied at node 1, expressed in node 1 frame.
        /// Note this is the torque applied on the body
        const Torque GetLinkReactionTorqueOnNode1(FRAME_CONVENTION fc) const;

        /// Get the link reaction torque applied at node 2, expressed in node 2 frame.
        /// Note this is the torque applied on the body
        const Torque GetLinkReactionTorqueOnNode2(FRAME_CONVENTION fc) const;

        /// Get the link reaction torque applied at body 1 COG, expressed in body 1 reference frame
        /// Note this is the torque applied on the body
        const Torque GetLinkReactionTorqueOnBody1AtCOG(FRAME_CONVENTION fc) const;

        /// Get the link reaction torque applied at body 2 COG, expressed in body 2 reference frame
        /// Note this is the torque applied on the body
        const Torque GetLinkReactionTorqueOnBody2AtCOG(FRAME_CONVENTION fc) const;

        /*
         * Link force in free degree of freedom (ie spring damping)
         */

        /// Get the force in link applying on body 1 at node 1 origin, expressed in frame 1
        const Force GetLinkForceOnBody1InFrame1AtOrigin1(FRAME_CONVENTION fc) const;

        /// Get the force in link applying on body 2 at node 2 origin, expressed in frame 2
        const Force GetLinkForceOnBody2InFrame2AtOrigin2(FRAME_CONVENTION fc) const;

        /// Get the torque in link applying on body 1 at node 1 origin, expressed in frame 1
        const Torque GetLinkTorqueOnBody1InFrame1AtOrigin1(FRAME_CONVENTION fc) const;

        /// Get the torque in link applying on body 2 at node 2 origin, expressed in frame 2
        const Torque GetLinkTorqueOnBody2InFrame2AtOrigin2(FRAME_CONVENTION fc) const;

        /// Get the force in link applying on body 1 at node 1 origin, expressed in frame 2
        const Force GetLinkForceOnBody1InFrame2AtOrigin1(FRAME_CONVENTION fc) const;

        /// Get the force in link applying on body 2 at node 2 origin, expressed in frame 1
        const Force GetLinkForceOnBody2InFrame1AtOrigin2(FRAME_CONVENTION fc) const;

        /// Get the torque in link applying on body 1 at node 1 origin, expressed in frame 2
        const Torque GetLinkTorqueOnBody1InFrame2AtOrigin1(FRAME_CONVENTION fc) const;

        /// Get the torque in link applying on body 2 at node 2 origin, expressed in frame 1
        const Torque GetLinkTorqueOnBody2InFrame1AtOrigin2(FRAME_CONVENTION fc) const;


        /// Generic computation of the power delivered in a FrLink
        virtual double GetLinkPower() const;

        /// Get the constraint violation of the link (ie the
        FrFrame GetConstraintViolation() const;  // FIXME : verifier que cette violation ne prend pas en compte la position relative normale de la liaison

        /// Initialize the link by setting the markers
        void Initialize() override;

        /// Update the link
        /// \param time time of the simulation
        virtual void Update(double time);


    protected:
        friend class FrNode; // To make possible to declare SetMarkers friend in FrNode

        /// Set the markers of the link. This method must be used during the Initialization of the link
        void SetNodes(FrNode* node1, FrNode* node2);

        /// Get the embedded Chrono object
        std::shared_ptr<chrono::ChLink> GetChronoLink() override;

        /// Get the internal item, as internal::FrLinkLockBase
        /// \return internal item, as internal::FrLinkLockBase
        internal::FrLinkLockBase* GetChronoItem_ptr() const override;

        /*
         * Methods allowing child classes to access chrono link forces
         */

        /// Set the link force expressed in marker 1 frame
        void SetLinkForceTorqueOnBody2InFrame2AtOrigin2(const Force &force, const Torque &torque);

//        /// Set the link torque expressed in marker 1 frame and applied at marker 1
//        void SetLinkTorqueOtMarker2InFrame2AtOrigin2(const Torque &torque);

        /// Update the cached values
        virtual void UpdateCache();

        /// Add the fields to the Hermes message
        void AddFields() override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRLINK_H
