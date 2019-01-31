//
// Created by frongere on 20/09/18.
//

#ifndef FRYDOM_FRLINK_H
#define FRYDOM_FRLINK_H


#include "frydom/core/common/FrPhysicsItem.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrConvention.h"
#include "frydom/core/common/FrFrame.h"


#include <memory>


// Forward declaration
namespace chrono {
    class ChLinkLock;
    class ChLinkMotor;
}


namespace frydom {

    // Forward declarations
    class FrOffshoreSystem_;
    class FrNode_;

    // TODO : mettre FrLinkBase dans un repertoire common. FrLink_ devra etre deplace dans le repertoire linkk_lib de
    // meme que les enums associes


    /*
     * FrLinkBase_
     */

    /// Pure abstract class for every FRyDoM constraints (FrLink_, FrConstraint_, FrActuator_)
    class FrLinkBase_ : public FrPhysicsItem_ {

    protected:

        std::shared_ptr<FrNode_> m_node1;   ///< the node on body 1 of the link
        std::shared_ptr<FrNode_> m_node2;   ///< the node on body 2 of the link

    public:
        FrLinkBase_(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);

        /// Tells if all constraints of this link are currently turned on or off by the user.
        virtual bool IsDisabled() const = 0;

        /// User can use this to enable/disable all the constraint of the link as desired.
        virtual void SetDisabled(bool disabled) = 0;

        /// Tells if the link is broken, for excess of pulling/pushing.
        virtual bool IsBroken() const = 0;

        /// Set the 'broken' status vof this link.
        virtual void SetBroken(bool broken) = 0;

        /// Tells if the link is currently active, in general,
        /// that is tells if it must be included into the system solver or not.
        /// This method cumulates the effect of various flags (so a link may
        /// be not active either because disabled, or broken, or not valid)
        virtual bool IsActive() const = 0;


        /// Returns the force in the link in 3 dof in the link frame 1 coordinate system
        virtual const Force GetLinkReactionForceInLinkFrame1() const = 0;

        /// Returns the force in the link in 3 dof in the link frame 2 coordinate system
        virtual const Force GetLinkReactionForceInLinkFrame2() const = 0;

        /// Returns the force in the link in 3 dof in the world frame coordinate system
        virtual const Force GetLinkReactionForceInWorldFrame(FRAME_CONVENTION fc) const = 0;


        /// Returns the torque in the link in 3 dof in the link frame coordinate system
        virtual const Torque GetLinkReactionTorqueInLinkFrame1() const = 0;

        /// Returns the torque in the link in 3 dof in the link frame coordinate system
        virtual const Torque GetLinkReactionTorqueInLinkFrame2() const = 0;

        /// Returns the torque in the link in 3 dof in the world frame coordinate system
        virtual const Torque GetLinkReactionTorqueInWorldFrame(FRAME_CONVENTION fc) const = 0;


        /// Returns the first node of the link
        FrNode_* GetNode1();

        /// Returns the second node of the link
        FrNode_* GetNode2();

        /// Returns the first body of the link
        FrBody_* GetBody1();

        /// Returns the second body of the link
        FrBody_* GetBody2();

    protected:  // TODO : voir si on rend cela private
        virtual void SetMarkers(FrNode_* node1, FrNode_* node2) = 0;

        friend void FrOffshoreSystem_::AddLink(std::shared_ptr<FrLinkBase_> link);
        virtual std::shared_ptr<chrono::ChLink> GetChronoLink() = 0;

        FrFrame_ GetTransformFromFrame2ToFrame1() const;


    };



    /// Different type of links implemented in FRyDO
    enum LINK_TYPE {
        CYLINDRICAL,
        FIXED_LINK,
        FREE_LINK,
        PRISMATIC,
        REVOLUTE,
//        SCREW,
        SPHERICAL
    };


    // Forward declaration
    class FrLink_;

    namespace internal {

        struct FrLinkLockBase : public chrono::ChLinkLock {

            FrLink_* m_frydomLink;

            /// Constructor
            explicit FrLinkLockBase(FrLink_* frydomLink);

            /// Set the link type based on LINK_TYPE enum
            void SetLinkType(LINK_TYPE lt);

            /// Initialize the link (calls the Initialize method of FrLink_
            void SetupInitial() override;

            /// Update (calls the ChLinkLock Update method then the FrLink_Update method
            void Update(double time, bool update_assets) override;

            /// Returns the marker 2 frame relatively to marker 1
            FrFrame_ GetRelativeFrame() const;

            /// Get the relative position of marker 2 with respect to 1, expressed in marker 1 frame
            Position GetRelativePosition() const;

            /// Get the relative orientation of frame 2 with respect to 1
            FrUnitQuaternion_ GetRelativeOrientation() const;

            /// Get the relative generalized velocity (stacked linear and angular vectors) of marker 2 with respect to
            /// marker 1, expressed in marker 1 frame
            GeneralizedVelocity GetRelativeGeneralizedVelocity() const;

            /// Get the velocity of marker 2 with respect to marker 1, expressed in marker 1 frame
            Velocity GetRelativeVelocity() const;

            /// Get the relative angular velocity of marker 2 with respect to marker 1, expressed in marker 1 frame
            AngularVelocity GetRelativeAngularVelocity() const;

            /// Get the relative generalized acceleration (stacked linear and angular vectors) of marker 2 with respect
            /// to marker 1, expressed in marker 1 frame
            GeneralizedAcceleration GetRelativeGeneralizedAcceleration() const;

            /// Get the relative acceleration of marker 2 with respect to marker 1, expressed in marker 1 frame
            Acceleration GetRelativeAcceleration() const;

            /// Get the relative angular acceleration of marker 2 with respect to marker 1, expressed in marker 1 frame
            AngularAcceleration GetRelativeAngularAcceleration() const;

        };

    }  // end namespace frydom::internal


    /*
     * FrLink_
     */

    class FrLink_ : public FrLinkBase_ {

    protected:
        std::shared_ptr<internal::FrLinkLockBase> m_chronoLink;

    public:
        FrLink_(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_ *system);

        bool IsDisabled() const override;

        void SetDisabled(bool disabled) override;

        bool IsBroken() const override;

        void SetBroken(bool broken) override;

        bool IsActive() const override;


        virtual void SetThisConfigurationAsReference();

        const Force GetLinkReactionForceInLinkFrame1() const override;

        const Force GetLinkReactionForceInLinkFrame2() const override;

        const Force GetLinkReactionForceInWorldFrame(FRAME_CONVENTION fc) const override;


        const Torque GetLinkReactionTorqueInLinkFrame1() const override;

        const Torque GetLinkReactionTorqueInLinkFrame2() const override;

        const Torque GetLinkReactionTorqueInWorldFrame(FRAME_CONVENTION fc) const override;

        void SetupInitial() override;

        void Initialize() override;

        void Update(double time) override;


    protected:
        friend class FrNode_; // To make possible to declare SetMarkers friend in FrNode_
        void SetMarkers(FrNode_* node1, FrNode_* node2) override;

        std::shared_ptr<chrono::ChLink> GetChronoLink() override;

    };


}  // end namespace frydom


#endif //FRYDOM_FRLINK_H
