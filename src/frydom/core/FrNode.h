//
// Created by frongere on 08/09/17.
//

#ifndef FRYDOM_FRNODE_H
#define FRYDOM_FRNODE_H

#include "chrono/physics/ChMarker.h"
#include "FrObject.h"
#include "FrVector.h"
#include "FrRotation.h"
#include "FrFrame.h"





namespace frydom {

//    class FrBody;

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













    /// REFACTORING ----------->>>>>>>>>>>>>>>>>


    // Forward declarations
    class FrBody_;
    class FrFrame_;


    class FrNode_ : public FrObject {

    private:

        FrBody_* m_body;
        std::shared_ptr<chrono::ChMarker> m_chronoMarker;

    public:

        /// Default Constructor
        /// \param body body to which the node belongs
        explicit FrNode_(FrBody_* body);

        /// Constructor from a Position
        /// \param body body to which the node belongs
        /// \param position relative position of the node in the body reference frame
        FrNode_(FrBody_* body, const Position& position);

        /// Constructor from a Position and a Rotation
        /// \param body body to which the node belongs
        /// \param position relative position of the frame node in the body reference frame
        /// \param rotation relative rotation of the frame node in the body reference frame
        FrNode_(FrBody_* body, const Position& position, const FrRotation_& rotation);

        /// Constructor from a Position and a Quaternion
        /// \param body body to which the node belongs
        /// \param position relative position of the frame node in the body reference frame
        /// \param quaternion relative rotation of the frame node in the body reference frame, given as quaternion
        FrNode_(FrBody_* body, const Position& position, const FrUnitQuaternion_& quaternion);

        /// Constructor from a frame
        /// \param body body to which the node belongs
        /// \param frame relative frame node, given in the body reference frame
        FrNode_(FrBody_* body, const FrFrame_& frame);

        /// Set node position and direction axis, given in the body reference frame
        /// \param body body to which the node belongs
        /// \param pos relative position of the frame node in the body reference frame
        /// \param e1 direction of the x-axis in the body reference frame
        /// \param e2 direction of the y-axis in the body reference frame
        /// \param e3 direction of the z-axis in the body reference frame
        void Set(FrBody_* body, Position pos, Direction e1, Direction e2, Direction e3);

        /// Destructor
        ~FrNode_();

        /// Get the body pointer
        /// \return the body to which the node belongs
        FrBody_* GetBody();

        /// Get the node frame, given in the world reference frame
        /// \return the node frame in the world reference frame
        FrFrame_ GetFrame() const;

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

        ///Initialize method not implmeented yet
        void Initialize() override;

        ///StepFinalize method not implmeented yet
        void StepFinalize() override;

    private:

        /// Set the node position, given in the body reference frame
        /// \param position node position, given in the body reference frame
        void SetLocalPosition(const Position& position);

        /// Set the node position, given in the body reference frame
        /// \param x x position, given in the body reference frame
        /// \param y y position, given in the body reference frame
        /// \param z z position, given in the body reference frame
        void SetLocalPosition(double x, double y, double z);

        /// Set the node rotation, given in the body reference frame
        /// \param quaternion rotation, as quaternion, given in the body reference frame
        void SetLocalQuaternion(const FrUnitQuaternion_& quaternion);

        /// Set the node rotation, given in the body reference frame
        /// \param rotation rotation, given in the body reference frame
        void SetLocalRotation(const FrRotation_& rotation);

        /// Set the node frame, given in the body reference frame
        /// \param frame frame, given in the body reference frame
        void SetLocalFrame(const FrFrame_& frame);


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
            return GetFrame().GetQuaternion().Rotate<Vector>(nodeVector, fc);
        }

        /// Project a vector from node reference frame to world reference frame
        /// \tparam Vector vector template
        /// \param nodeVector vector to be projected, given in the node reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the vector projected in the world reference frame
        template <class Vector>
        Vector& ProjectVectorInWorld(Vector& nodeVector, FRAME_CONVENTION fc) const {
            nodeVector = GetFrame().GetQuaternion().Rotate<Vector>(nodeVector, fc);
            return nodeVector;
        }

        /// Project a vector from world reference frame to node reference frame
        /// \tparam Vector vector template
        /// \param nodeVector vector to be projected, given in the world reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the vector projected in the node reference frame
        template <class Vector>
        Vector ProjectVectorInNode(const Vector &worldVector, FRAME_CONVENTION fc) const {
            return GetFrame().GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
        }

        /// Project a vector from world reference frame to node reference frame
        /// \tparam Vector vector template
        /// \param nodeVector vector to be projected, given in the world reference frame
        /// \param fc Frame convention (NED/NWU)
        /// \return the vector projected in the node reference frame
        template <class Vector>
        Vector& ProjectVectorInNode(Vector& worldVector, FRAME_CONVENTION fc) const {
            worldVector = GetFrame().GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
            return worldVector;
        }


    };








}  // end namespace frydom


#endif //FRYDOM_FRNODE_H
