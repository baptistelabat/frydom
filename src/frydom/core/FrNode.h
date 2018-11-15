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

        explicit FrNode_(FrBody_* body);

        FrNode_(FrBody_* body, const Position& position);

        FrNode_(FrBody_* body, const Position& position, const FrRotation_& rotation);

        FrNode_(FrBody_* body, const Position& position, const FrQuaternion_& quaternion);

        FrNode_(FrBody_* body, const FrFrame_& frame);

        ~FrNode_();

        FrBody_* GetBody();

        FrFrame_ GetFrame() const;

        Position GetPositionInWorld(FRAME_CONVENTION fc) const;

        void GetPositionInWorld(Position &position, FRAME_CONVENTION fc);

        Position GetNodePositionInBody(FRAME_CONVENTION fc) const;

        Velocity GetVelocityInWorld(FRAME_CONVENTION fc) const;

        Velocity GetVelocityInNode(FRAME_CONVENTION fc) const;

        Acceleration GetAccelerationInWorld(FRAME_CONVENTION fc) const;

        Acceleration GetAccelerationInNode(FRAME_CONVENTION fc) const;

        void Initialize() override;

        void StepFinalize() override;

    private:

        void SetLocalPosition(const Position& position);

        void SetLocalPosition(double x, double y, double z);

        void SetLocalQuaternion(const FrQuaternion_& quaternion);

        void SetLocalRotation(const FrRotation_& rotation);

        void SetLocalFrame(const FrFrame_& frame);


        // =============================================================================================================
        // PROJECTIONS
        // =============================================================================================================

        // Projection of 3D vectors defined in FrVector.h

        template <class Vector>
        Vector ProjectVectorInWorld(const Vector& bodyVector, FRAME_CONVENTION fc) const {
            return GetFrame().GetQuaternion().Rotate<Vector>(bodyVector, fc);
        }

        template <class Vector>
        Vector& ProjectVectorInWorld(Vector& bodyVector, FRAME_CONVENTION fc) const {
            bodyVector = GetFrame().GetQuaternion().Rotate<Vector>(bodyVector, fc);
            return bodyVector;
        }

        template <class Vector>
        Vector ProjectVectorInNode(const Vector &worldVector, FRAME_CONVENTION fc) const {
            return GetFrame().GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
        }

        template <class Vector>
        Vector& ProjectVectorInNode(Vector& worldVector, FRAME_CONVENTION fc) const {
            worldVector = GetFrame().GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
            return worldVector;
        }


    };








}  // end namespace frydom


#endif //FRYDOM_FRNODE_H
