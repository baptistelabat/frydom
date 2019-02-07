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


#ifndef FRYDOM_FRNODE_H
#define FRYDOM_FRNODE_H

#include "chrono/physics/ChMarker.h"
#include "FrObject.h"
#include "frydom/core/math/FrVector.h"
#include "FrRotation.h"
#include "FrFrame.h"

#include "frydom/core/link/links_lib/FrLink.h"


namespace frydom {

//    class FrBody;

    /**
     * \class FrNode
     * \brief Class for defining nodes (in order to add links).
     */
    class FrNode : public chrono::ChMarker, public FrObject {
//    private:
//        FrBody* Body;

    public:

//        FrBody* GetBody() const { return Body; }

        FrNode() : chrono::ChMarker() {
//            SetMotionType(M_MOTION_EXTERNAL); // Hack to keep the relative position unchanged
        }

//        FrNode(char myname[], FrBody* myBody,
//               chrono::Coordsys myrel_pos, chrono::Coordsys myrel_pos_dt, chrono::Coordsys myrel_pos_dtdt)
//            : chrono::ChMarker(myname, myBody, myrel_pos, myrel_pos_dt, myrel_pos_dtdt) {
//            SetMotionType(M_MOTION_EXTERNAL); // Hack to keep the relative position unchanged
//        }

        chrono::ChVector<double> GetAbsPos() {
            return GetAbsCoord().pos;
        }

//        std::shared_ptr<FrBody> GetSharedBody() {
//            return Body->shared_from_this();
//        }

        chrono::Coordsys GetRelPos() const {
            return GetRest_Coord();
        }

        virtual void Initialize() override {}

        virtual void StepFinalize() override {}

    };  // end class FrNode













    // REFACTORING ----------->>>>>>>>>>>>>>>>>


    // Forward declarations
    class FrBody_;
    class FrFrame_;
    class FrNode_;

    namespace internal{

    struct _FrMarkerBase_ : public chrono::ChMarker {
        FrNode_ * m_frydomNode;

        _FrMarkerBase_(FrNode_* node) {m_frydomNode = node;}
    };


    }

    /**
     * \class FrNode_
     * \brief Class for defining nodes (in order to add links).
     */
    class FrNode_ : public FrObject {

    private:

        FrBody_* m_body;                                    ///< Pointer to the body containing this node
        std::shared_ptr<chrono::ChMarker> m_chronoMarker;   ///< Chrono class for nodes/marker.

    public:

        /// Default Constructor
        /// \param body body to which the node belongs
        explicit FrNode_(FrBody_* body);

//        /// Constructor from a Position
//        /// \param body body to which the node belongs
//        /// \param position relative position of the node with respect to body reference frame
//        FrNode_(FrBody_* body, const Position& position, FRAME_CONVENTION fc);
//
//        /// Constructor from a Position and a Rotation
//        /// \param body body to which the node belongs
//        /// \param position relative position of the frame node with respect to body reference frame
//        /// \param rotation relative rotation of the frame node with respect to body reference frame
//        FrNode_(FrBody_* body, const Position& position, const FrRotation_& rotation, FRAME_CONVENTION fc);
//
//        /// Constructor from a Position and a Quaternion
//        /// \param body body to which the node belongs
//        /// \param position relative position of the frame node with respect to body reference frame
//        /// \param quaternion relative rotation of the frame node with respect to body reference frame, given as quaternion
//        FrNode_(FrBody_* body, const Position& position, const FrUnitQuaternion_& quaternion, FRAME_CONVENTION fc);
//
//        /// Constructor from a frame
//        /// \param body body to which the node belongs
//        /// \param frame relative frame node, with respect to body reference frame
//        FrNode_(FrBody_* body, const FrFrame_& frame);

        /// Set node position and direction axis, with respect to body reference frame
        /// \param pos relative position of the frame node with respect to body reference frame
        /// \param e1 direction of the x-axis with respect to reference frame
        /// \param e2 direction of the y-axis with respect to reference frame
        /// \param e3 direction of the z-axis with respect to reference frame
        void Set(const Position& pos, const Direction& e1, const Direction& e2, const Direction& e3, FRAME_CONVENTION fc);


        /*
         * Setters
         */

        /// Set the node position with respect to body reference frame
        void SetPositionInBody(const Position& bodyPosition, FRAME_CONVENTION fc);
        void SetPositionInWorld(const Position& worldPosition, FRAME_CONVENTION fc);

        void TranslateInBody(const Translation &translationInBody, FRAME_CONVENTION fc);
        void TranslateInBody(const Direction& directionBody, double distance, FRAME_CONVENTION fc);
        void TranslateInBody(double x, double y, double z, FRAME_CONVENTION fc);

        void TranslateInWorld(const Translation &translationInWorld, FRAME_CONVENTION fc);
        void TranslateInWorld(const Direction& directionWorld, double distance, FRAME_CONVENTION fc);
        void TranslateInWorld(double x, double y, double z, FRAME_CONVENTION fc);


        void SetOrientationInBody(const FrRotation_& rotation);
        void SetOrientationInBody(const FrUnitQuaternion_& quaternion);

        void RotateInBody(const FrRotation_& rotation);
        void RotateInBody(const FrUnitQuaternion_& quaternion);

        void RotateInWorld(const FrRotation_& rotation);
        void RotateInWorld(const FrUnitQuaternion_& quaternion);

        void RotateAroundXInBody(double angleRad, FRAME_CONVENTION fc);
        void RotateAroundYInBody(double angleRad, FRAME_CONVENTION fc);
        void RotateAroundZInBody(double angleRad, FRAME_CONVENTION fc);

        void RotateAroundXInWorld(double angleRad, FRAME_CONVENTION fc);
        void RotateAroundYInWorld(double angleRad, FRAME_CONVENTION fc);
        void RotateAroundZInWorld(double angleRad, FRAME_CONVENTION fc);



        /// Destructor
        ~FrNode_() = default;

        /// Get the body pointer
        /// \return the body to which the node belongs
        FrBody_* GetBody();

        /// Get the node frame, given in the world reference frame
        /// \return the node frame in the world reference frame
        FrFrame_ GetFrameInWorld() const;

        /// Get the node frame, given in the body reference frame
        /// \return the node frame in the body reference frame
        FrFrame_ GetFrameInBody() const;

        /// Get the node frame with respect to the COG in body reference coordinates
        /// \return the node frame with respect to COG
        FrFrame_ GetFrameWRT_COG_InBody() const;

        void SetFrameInBody(const FrFrame_& frameInBody);

        void SetFrameInWorld(const FrFrame_& frameInWorld);


        /// Get the node position in world reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the node position in world reference frame
        Position GetPositionInWorld(FRAME_CONVENTION fc) const;

        /// Get the node position in world reference frame
        /// \param position reference to the node position in world reference frame
        /// \param fc Frame convention (NED/NWU)
        void GetPositionInWorld(Position &position, FRAME_CONVENTION fc);

        /// Get the node position in body reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the node position in body reference frame
        Position GetNodePositionInBody(FRAME_CONVENTION fc) const;

        /// Get the node velocity in world reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the node velocity in world reference frame
        Velocity GetVelocityInWorld(FRAME_CONVENTION fc) const;

        /// Get the node velocity in the node reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the node velocity in the node reference frame
        Velocity GetVelocityInNode(FRAME_CONVENTION fc) const;

        /// Get the node acceleration in world reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the node acceleration in world reference frame
        Acceleration GetAccelerationInWorld(FRAME_CONVENTION fc) const;

        /// Get the node acceleration in the node reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the node acceleration in the node reference frame
        Acceleration GetAccelerationInNode(FRAME_CONVENTION fc) const;

        /// Initialize method not implemented yet
        void Initialize() override;

        /// StepFinalize method not implemented yet
        void StepFinalize() override;



        // =============================================================================================================
        // PROJECTIONS
        // =============================================================================================================

        // Projection of 3D vectors defined in FrVector.h

        /// Project a vector from node reference frame to world reference frame
        /// \tparam Vector vector template
        /// \param nodeVector vector to be projected, given in the node reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the vector projected in the world reference frame
        template <class Vector>
        Vector ProjectVectorInWorld(const Vector& nodeVector, FRAME_CONVENTION fc) const {
            return GetFrameInWorld().GetQuaternion().Rotate<Vector>(nodeVector, fc);
        }

        /// Project a vector from node reference frame to world reference frame
        /// \tparam Vector vector template
        /// \param nodeVector vector to be projected, given in the node reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the vector projected in the world reference frame
        template <class Vector>
        Vector& ProjectVectorInWorld(Vector& nodeVector, FRAME_CONVENTION fc) const {
            nodeVector = GetFrameInWorld().GetQuaternion().Rotate<Vector>(nodeVector, fc);
            return nodeVector;
        }

        /// Project a vector from world reference frame to node reference frame
        /// \tparam Vector vector template
        /// \param nodeVector vector to be projected, given in the world reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the vector projected in the node reference frame
        template <class Vector>
        Vector ProjectVectorInNode(const Vector &worldVector, FRAME_CONVENTION fc) const {
            return GetFrameInWorld().GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
        }

        /// Project a vector from world reference frame to node reference frame
        /// \tparam Vector vector template
        /// \param nodeVector vector to be projected, given in the world reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the vector projected in the node reference frame
        template <class Vector>
        Vector& ProjectVectorInNode(Vector& worldVector, FRAME_CONVENTION fc) const {
            worldVector = GetFrameInWorld().GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
            return worldVector;
        }

    private:
        /*
         * The following methods are for internal use only. Everything must be passed as NWU referenced values !
         */

//        /// Set the node position, with respect to the body reference frame
//        /// \param position node position, with respect to the body reference frame
//        void SetLocalPosition(const Position& position);
//
//        /// Set the node position, given in the body reference frame
//        /// \param x x position, with respect to the body reference frame
//        /// \param y y position, with respect to the body reference frame
//        /// \param z z position, with respect to the body reference frame
//        void SetLocalPosition(double x, double y, double z);
//
////        void SetWorldPosition(const Position& position);  // TODO : tester !!!!
//
//
//
//
//        /// Set the node rotation, with respect to the body reference frame
//        /// \param quaternion rotation, as quaternion, with respect to the body reference frame
//        void SetLocalQuaternion(const FrUnitQuaternion_& quaternion);
//
//        /// Set the node rotation, with respect to the body reference frame
//        /// \param rotation rotation, with respect to the body reference frame
//        void SetLocalRotation(const FrRotation_& rotation);


        // TODO : voir si on garde les 2 methodes suivantes privees...

        /// Set the node frame, with respect to the body reference frame
        /// \param frame frame, with respect to the body reference frame
//        void _SetLocalFrame(const FrFrame_ &frame);
//
//        void _SetWorldFrame(const FrFrame_ &frame);


        friend void FrLink_::SetMarkers(FrNode_*, FrNode_*);
//        friend void FrActuator_::SetMarkers(FrNode_*, FrNode_*);

    };




}  // end namespace frydom


#endif //FRYDOM_FRNODE_H
