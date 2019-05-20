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


// Forward declaration
namespace chrono {
    class ChLinkLock;
    class ChLinkMotor;
}


namespace frydom {

    // Forward declarations
    class FrOffshoreSystem;
    class FrNode;

    // TODO : mettre FrLinkBase dans un repertoire common. FrLink devra etre deplace dans le repertoire linkk_lib de
    // meme que les enums associes

    /// Different type of links implemented in FRyDO
    enum LINK_TYPE {
        CYLINDRICAL,
        FIXED_LINK,
        FREE_LINK,
        PRISMATIC,
        REVOLUTE,
        SPHERICAL,
        PERPENDICULAR,
        PARALLEL,
        PLANEONPLANE,
        POINTONLINE,
        POINTONPLANE,
        CUSTOM
//        SCREW,
//        POINTONSPLINE,
//        DISTANCETOAXIS,
    };

 /*
 * Defining a mask class to make the constraint on bodies WRT to world easier
 */
    // TODO : creer une classe speciale derivant de FrLink pour fixer les dof des corps en mode captif..
    // FrBodyCaptive... --> on mettra le IsMotorized en virtuel pur !

    /**
     * \class FrDOFMask
     * \brief Class for defining the constraints on bodies with respect to world easier.
     */
    class FrDOFMask {

    private:

        LINK_TYPE m_linkType = FREE_LINK;
        bool m_xLocked = false;
        bool m_yLocked = false;
        bool m_zLocked = false;
        bool m_RxLocked = false;
        bool m_RyLocked = false;
        bool m_RzLocked = false;

    public:

        FrDOFMask();

        // TODO : plutot utiliser ChLinkLockMaskLF en interne et faire des conversions pour les angles vers les coeffs de quaternion
        /*
         * Pour les angles, un blocage en
         */

        /// If true, locks the X DOF of the body
        void SetLock_X(bool lock);

        /// If true, locks the Y DOF of the body
        void SetLock_Y(bool lock);

        /// If true, locks the Z DOF of the body
        void SetLock_Z(bool lock);

        /// If true, locks the RX DOF of the body
        void SetLock_Rx(bool lock);

        /// If true, locks the RY DOF of the body
        void SetLock_Ry(bool lock);

        /// If true, locks the RZ DOF of the body
        void SetLock_Rz(bool lock);

        /// Locking the body in the world vertical plane
        void LockXZPlane();  // On bloque y, rx, rz

        /// Locking the body in the world horizontal plane
        void LockXYPlane();  // On bloque z, ry

        /// Is the X DOF locked ?
        bool GetLock_X() const;

        /// Is the Y DOF locked ?
        bool GetLock_Y() const;

        /// Is the Z DOF locked ?
        bool GetLock_Z() const;

        /// Is the RX DOF locked ?
        bool GetLock_Rx() const;

        /// Is the RY DOF locked ?
        bool GetLock_Ry() const;

        /// Is the RZ DOF locked ?
        bool GetLock_Rz() const;

        /// Returns true if the body has some locked DOF
        bool HasLockedDOF() const;

        /// Returns true if the body has no embedded constraint
        bool IsFree() const;

        /// Removes every constraint from the body WRT to the world
        void MakeItFree();

        /// Makes the body fixed in the world
        void MakeItLocked();

        /// Returns the number of locked DOF of the body WRT the world
        unsigned int GetNbLockedDOF() const;

        /// Returns the number of constrained DOF of the body WRT the world
        unsigned int GetNbFreeDOF() const;

        void SetLinkType(LINK_TYPE linkType);

        LINK_TYPE GetLinkType() const;

    private:

        void SetLock(bool xLocked, bool yLocked, bool zLocked, bool rxLocked, bool ryLocked, bool rzLocked) ;

    };

    // Forward declaration
    class FrLink;

    namespace internal {

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

            GeneralizedForce c_generalizedForceOnMarker2;
            GeneralizedForce c_generalizedForceOnMarker1;


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

            /// Set a mask. Mainly used in the case of body constraints with the world where we do not use specialized
            // classes for the link
            void SetMask(FrDOFMask* mask); // TODO : ne pas avoir de reference a body dof mask ici

            /// Set the link force applying on Body 1 (as external) expressed in marker 2 frame and on marker 1 origin
            void SetLinkForceOnBody1InFrame2AtOrigin1(const Force &force);

            /// Set the link torque applying on Body 1 (as external) expressed in marker 2 frame and on marker 1 origin
            void SetLinkTorqueOnBody1InFrame2AtOrigin1(const Torque& torque);

            /// Get the link force applying on Body 1 (as external) expressed in marker 2 frame and on marker 1 origin
            Force GetLinkForceOnBody1InFrame2AtOrigin1();

            /// Get the link torque applying on Body 1 (as external) expressed in marker 2 frame and on marker 1 origin
            Torque GetLinkTorqueOnBody1InFrame2ArOrigin1();

            FrFrame GetConstraintViolation();

        };

    }  // end namespace frydom::internal



    /*
     * FrLink
     */

    // Forward declaration
    class FrActuator;

    /**
     * \class FrLink
     * \brief Class to deal with links.
     */
    class FrLink : public FrLinkBase {

    protected:

        std::shared_ptr<internal::FrLinkLockBase> m_chronoLink;

        FrFrame m_frame2WRT1_reference;
        bool m_breakable = false; // TODO : utiliser et permettre de declencher la cassure de la liaison

        FrDOFMask m_dofMask;

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

        virtual void SetLocked(bool locked);

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

        /// Get the Marker 2 frame relatively to marker 1 frame
        const FrFrame GetMarker2FrameWRTMarker1Frame() const;

        /// Get the Marker 1 frame relatively to marker 2 frame
        const FrFrame GetMarker1FrameWRTMarker2Frame() const;

        /// Get the Marker 2 position relatively to marker 1, expressed in marker 1 frame
        const Position GetMarker2PositionWRTMarker1(FRAME_CONVENTION fc) const;

        /// Get the Marker 1 position relatively to marker 2, expressed in marker 2 frame
        const Position GetMarker1PositionWRTMarker2(FRAME_CONVENTION fc) const;

        /// Get the Marker 2 orientation relatively to marker 1
        const FrRotation GetMarker2OrientationWRTMarker1() const;

        /// Get the Marker 1 orientation relatively to marker 2
        const FrRotation GetMarker1OrientationWRTMarker2() const;

        /*
         * Velocity related methods
         */

        /// Get the Marker 2 generalized velocity with respect to marker 1, expressed in marker 1 frame
        const GeneralizedVelocity GetGeneralizedVelocityOfMarker2WRTMarker1(FRAME_CONVENTION fc) const;

        /// Get the Marker 1 generalized velocity with respect to marker 2, expressed in marker 2 frame
        const GeneralizedVelocity GetGeneralizedVelocityOfMarker1WRTMarker2(FRAME_CONVENTION fc) const;

        /// Get the Marker 2 velocity with respect to marker 1, expressed in marker 1 frame
        const Velocity GetVelocityOfMarker2WRTMarker1(FRAME_CONVENTION fc) const;

        /// Get the Marker 1 velocity with respect to marker 2, expressed in marker 2 frame
        const Velocity GetVelocityOfMarker1WRTMarker2(FRAME_CONVENTION fc) const;

        /// Get the Marker 2 angular velocity with respect to marker 1, expressed in marker 1 frame
        const AngularVelocity GetAngularVelocityOfMarker2WRTMarker1(FRAME_CONVENTION fc) const;

        /// Get the Marker 1 angular velocity with respect to marker 2, expressed in marker 2 frame
        const AngularVelocity GetAngularVelocityOfMarker1WRTMarker2(FRAME_CONVENTION fc) const;

        /*
         * Acceleration related methods
         */

        /// Get the Marker 2 generalized acceleration with respect to marker 1, expressed in marker 1 frame
        const GeneralizedAcceleration GetGeneralizedAccelerationOfMarker2WRTMarker1(FRAME_CONVENTION fc) const;

        /// Get the Marker 1 generalized acceleration with respect to marker 2, expressed in marker 2 frame
        const GeneralizedAcceleration GetGeneralizedAccelerationOfMarker1WRTMarker2(FRAME_CONVENTION fc) const;

        /// Get the Marker 2 acceleration with respect to marker 1, expressed in marker 1 frame
        const Acceleration GetAccelerationOfMarker1WRTMarker2(FRAME_CONVENTION fc) const;

        /// Get the Marker 1 acceleration with respect to marker 2, expressed in marker 2 frame
        const Acceleration GetAccelerationOfMarker2WRTMarker1(FRAME_CONVENTION fc) const;

        /// Get the Marker 2 angular acceleration with respect to marker 1, expressed in marker 1 frame
        const AngularAcceleration GetAngularAccelerationOfMarker2WRTMarker1(FRAME_CONVENTION fc) const;

        /// Get the Marker 1 angular acceleration with respect to marker 2, expressed in marker 2 frame
        const AngularAcceleration GetAngularAccelerationOfMarker1WRTMarker2(FRAME_CONVENTION fc) const;

        /*
         * Force related methods
         */

        /// Get the link reaction force applied at marker 1, expressed in marker 1 frame.
        /// Note this is the force applied on the body
        const Force GetLinkReactionForceOnMarker1(FRAME_CONVENTION fc) const;

        /// Get the link reaction force applied at marker 2, expressed in marker 2 frame
        /// Note this is the force applied on the body
        const Force GetLinkReactionForceOnMarker2(FRAME_CONVENTION fc) const;

        /// Get the link reaction force applied at marker 1, expressed in body 1 frame.
        /// Note this is the force applied on the body
        const Force GetLinkReactionForceOnBody1(FRAME_CONVENTION fc) const;

        /// Get the link reaction force applied at marker 2, expressed in body 2 frame
        /// Note this is the force applied on the body
        const Force GetLinkReactionForceOnBody2(FRAME_CONVENTION fc) const;

        /// Get the link reaction torque applied at marker 1, expressed in marker 1 frame.
        /// Note this is the torque applied on the body
        const Torque GetLinkReactionTorqueOnMarker1(FRAME_CONVENTION fc) const;

        /// Get the link reaction torque applied at marker 2, expressed in marker 2 frame.
        /// Note this is the torque applied on the body
        const Torque GetLinkReactionTorqueOnMarker2(FRAME_CONVENTION fc) const;

        /// Get the link reaction torque applied at body 1 COG, expressed in body 1 reference frame
        /// Note this is the torque applied on the body
        const Torque GetLinkReactionTorqueOnBody1AtCOG(FRAME_CONVENTION fc) const;

        /// Get the link reaction torque applied at body 2 COG, expressed in body 2 reference frame
        /// Note this is the torque applied on the body
        const Torque GetLinkReactionTorqueOnBody2AtCOG(FRAME_CONVENTION fc) const;

        /*
         * Link force in free degree of freedom (ie spring damping)
         */

        /// Get the force in link applying on body 1 at marker 1 origin, expressed in frame 1
        const Force GetLinkForceOnBody1InFrame1AtOrigin1(FRAME_CONVENTION fc) const;

        /// Get the force in link applying on body 2 at marker 2 origin, expressed in frame 2
        const Force GetLinkForceOnBody2InFrame2AtOrigin2(FRAME_CONVENTION fc) const;

        /// Get the torque in link applying on body 1 at marker 1 origin, expressed in frame 1
        const Torque GetLinkTorqueOnBody1InFrame1AtOrigin1(FRAME_CONVENTION fc) const;

        /// Get the torque in link applying on body 2 at marker 2 origin, expressed in frame 2
        const Torque GetLinkTorqueOnBody2InFrame2AtOrigin2(FRAME_CONVENTION fc) const;

        /// Get the force in link applying on body 1 at marker 1 origin, expressed in frame 2
        const Force GetLinkForceOnBody1InFrame2AtOrigin1(FRAME_CONVENTION fc) const;

        /// Get the force in link applying on body 2 at marker 2 origin, expressed in frame 1
        const Force GetLinkForceOnBody2InFrame1AtOrigin2(FRAME_CONVENTION fc) const;

        /// Get the torque in link applying on body 1 at marker 1 origin, expressed in frame 2
        const Torque GetLinkTorqueOnBody1InFrame2AtOrigin1(FRAME_CONVENTION fc) const;

        /// Get the torque in link applying on body 2 at marker 2 origin, expressed in frame 1
        const Torque GetLinkTorqueOnBody2InFrame1AtOrigin2(FRAME_CONVENTION fc) const;


        /// Generic computation of the power delivered in a FrLink
        virtual double GetLinkPower() const;

        /// Initialize a FrLink with a FrDOFMask. Essentially used by the DOF restricting mechanism of bodies
        /// Users should not use this method to make links between bodies but directly use the specialized classes
        /// (FrPrismaticLink, FrRevoluteLink...)
        void InitializeWithBodyDOFMask(FrDOFMask* mask); // FIXME passer dans une classe dediee specialisee FrBodyCaptive

        /// Get the constraint violation of the link (ie the
        FrFrame GetConstraintViolation() const;  // FIXME : verifier que cette violation ne prend pas en compte la position relative normale de la liaison

        void Initialize() override;

        virtual void Update(double time);


    protected:
        friend class FrNode; // To make possible to declare SetMarkers friend in FrNode

        /// Set the markers of the link. This method must be used during the Initialization of the link
        void SetMarkers(FrNode* node1, FrNode* node2);

        /// Get the embedded Chrono object
        std::shared_ptr<chrono::ChLink> GetChronoLink() override;

        internal::FrLinkLockBase* GetChronoItem_ptr() const override;

        /*
         * Methods allowing child classes to access chrono link forces
         */

        /// Set the link force expressed in marker 1 frame
        void SetLinkForceTorqueOnBody2InFrame2AtOrigin2(const Force &force, const Torque &torque);

//        /// Set the link torque expressed in marker 1 frame and applied at marker 1
//        void SetLinkTorqueOtMarker2InFrame2AtOrigin2(const Torque &torque);
        virtual void UpdateCache();

        void AddFields() override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRLINK_H
