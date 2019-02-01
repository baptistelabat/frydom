//
// Created by frongere on 20/09/18.
//

#ifndef FRYDOM_FRLINK_H
#define FRYDOM_FRLINK_H


//#include "frydom/core/common/FrPhysicsItem.h"
//#include "frydom/core/math/FrVector.h"
//#include "frydom/core/common/FrConvention.h"
//#include "frydom/core/common/FrFrame.h"

#include "frydom/core/link/FrLinkBase.h"


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

            using ChronoLinkType = chrono::ChLinkLock::LinkType;

            FrLink_* m_frydomLink; ///> Pointer to frydom link object container

            // Cache data for performance
            FrFrame_ c_frame1WRT2;
            FrFrame_ c_frame2WRT1;

            GeneralizedVelocity c_generalizedVelocity1WRT2;
            GeneralizedVelocity c_generalizedVelocity2WRT1;

            GeneralizedAcceleration c_generalizedAcceleration1WRT2;
            GeneralizedAcceleration c_generalizedAcceleration2WRT1;

            GeneralizedForce c_generalizedForceOnMarker2;
            GeneralizedForce c_generalizedForceOnMarker1;


            /// Constructor
            explicit FrLinkLockBase(FrLink_* frydomLink);

            /// Set the link type based on LINK_TYPE enum
            void SetLinkType(LINK_TYPE lt);

            /// Initialize the link (calls the Initialize method of FrLink_
            void SetupInitial() override;

            /// Update (calls the ChLinkLock Update method then the FrLink_Update method
            void Update(double time, bool update_assets) override;

            void GenerateCache();

            /// Set the link force applying on Body 2 in marker 2 frame
            void SetLinkForceOnBody1InFrame2AtOrigin1(const Force &force) {
                C_force = internal::Vector3dToChVector(force);
            }

            void SetLinkTorqueOnBody1InFrame2AtOrigin1(const Torque& torque) {
                C_torque = internal::Vector3dToChVector(torque);
            }

            Force GetLinkForceOnBody1InFrame2AtOrigin1() {
                return internal::ChVectorToVector3d<Force>(C_force);
            }

            Torque GetLinkTorqueOnBody1InFrame2ArOrigin1() {
                return internal::ChVectorToVector3d<Torque>(C_torque);
            }

        };

    }  // end namespace frydom::internal


    /*
     * FrLink_
     */

    class FrLink_ : public FrLinkBase_ {

    protected:

        std::shared_ptr<internal::FrLinkLockBase> m_chronoLink;

        FrFrame_ m_frame2WRT1_reference;

    public:

        /// Constructor taking the nodes attached to the two bodies implied in the link and the system
        FrLink_(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_ *system);

        /// Tells if all constraints of this link are currently turned on or off by the user.
        bool IsDisabled() const override;

        /// User can use this to enable/disable all the constraint of the link as desired.
        void SetDisabled(bool disabled) override;

        /// Tells if the link is broken, for excess of pulling/pushing. // TODO : implementer de quoi 'casser' les liaisons si trop d'effort -> foncteurs ?
        bool IsBroken() const override;

        /// Set the 'broken' status vof this link.
        void SetBroken(bool broken) override;

        /// Tells if the link is currently active, in general,
        /// that is tells if it must be included into the system solver or not.
        /// This method cumulates the effect of various flags (so a link may
        /// be not active either because disabled, or broken, or not valid)
        bool IsActive() const override;

        /// Freezes the current configuration as being the reference configuration from which the generalized
        /// position measurement are given (ie. length from the rest length for a prismatic link)
        virtual void SetThisConfigurationAsReference();


        /*
         * Position related methods
         */

        /// Get the Marker 2 frame relatively to marker 1 frame
        const FrFrame_ GetMarker2FrameWRTMarker1Frame() const;

        /// Get the Marker 1 frame relatively to marker 2 frame
        const FrFrame_ GetMarker1FrameWRTMarker2Frame() const;

        /// Get the Marker 2 position relatively to marker 1, expressed in marker 1 frame
        const Position GetMarker2PositionWRTMarker1(FRAME_CONVENTION fc) const;

        /// Get the Marker 1 position relatively to marker 2, expressed in marker 2 frame
        const Position GetMarker1PositionWRTMarker2(FRAME_CONVENTION fc) const;

        /// Get the Marker 2 orientation relatively to marker 1
        const FrRotation_ GetMarker2OrientationWRTMarker1() const;

        /// Get the Marker 1 orientation relatively to marker 2
        const FrRotation_ GetMarker1OrientationWRTMarker2() const;

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


        virtual double GetLinkPower() const;


        virtual void Initialize() override;

        virtual void Update(double time) override;


    protected:
        friend class FrNode_; // To make possible to declare SetMarkers friend in FrNode_

        /// Set the markers of the link. This method must be used during the Initialization of the link
        void SetMarkers(FrNode_* node1, FrNode_* node2) override;

        /// Get the embedded Chrono object
        std::shared_ptr<chrono::ChLink> GetChronoLink() override;

        /*
         * Methods allowing child classes to access chrono link forces
         */

        /// Set the link force expressed in marker 1 frame
        void SetLinkForceOnBody2InFrame2AtOrigin2(const Force &force, const Torque& torque);

//        /// Set the link torque expressed in marker 1 frame and applied at marker 1
//        void SetLinkTorqueOtMarker2InFrame2AtOrigin2(const Torque &torque);









    };


}  // end namespace frydom


#endif //FRYDOM_FRLINK_H
