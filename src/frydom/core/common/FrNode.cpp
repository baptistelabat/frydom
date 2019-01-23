//
// Created by frongere on 08/09/17.
//

#include "FrNode.h"
#include "frydom/core/body/FrBody.h"

#include "frydom/core/math/FrMatrix.h"


namespace frydom {


    FrNode_::FrNode_(frydom::FrBody_ *body) : m_body(body) {
        m_chronoMarker = std::make_shared<chrono::ChMarker>();
        body->GetChronoBody()->AddMarker(m_chronoMarker);  //Chrono body can be retrieved because this constructor is a friend of FrBody_
    }

    FrNode_::FrNode_(FrBody_ *body, const Position &position) : FrNode_(body) {
        SetLocalPosition(position);
    }

    FrNode_::FrNode_(FrBody_ *body, const Position &position, const FrRotation_ &rotation) : FrNode_(body) {
        SetLocalFrame( FrFrame_(position, rotation, NWU) );
    }

    FrNode_::FrNode_(FrBody_ *body, const Position &position, const FrUnitQuaternion_ &quaternion) : FrNode_(body) {
        SetLocalFrame( FrFrame_(position, quaternion, NWU) );
    }

    FrNode_::FrNode_(FrBody_ *body, const FrFrame_ &frame) : FrNode_(body) {
        SetLocalFrame(frame);
    }

    void FrNode_::Set(FrBody_* body, Position pos, Direction e1, Direction e2, Direction e3) {
        m_body = body;
        SetLocalPosition(pos);

        mathutils::Matrix33<double> matrix;                 // FIXME : passer un FrRotation plutôt que matrix33
        matrix <<   e1.Getux(), e2.Getux(), e3.Getux(),
                    e1.Getuy(), e2.Getuy(), e3.Getuy(),
                    e1.Getuz(), e2.Getuz(), e3.Getuz();
        m_chronoMarker->SetRot(internal::Matrix33ToChMatrix33(matrix));
        m_chronoMarker->UpdateState();
    }

    FrNode_::~FrNode_() = default;

    FrBody_* FrNode_::GetBody() {
        return m_body;
    }

    FrFrame_ FrNode_::GetFrame() const {
        return internal::Ch2FrFrame(m_chronoMarker->GetAbsFrame());
    }

    void FrNode_::SetLocalPosition(const Position &position) {
        Position localPositionFromCOG = position - m_body->GetCOG(NWU);
        auto coord = chrono::ChCoordsys<double>(chrono::ChVector<double>(internal::Vector3dToChVector(localPositionFromCOG)));
        m_chronoMarker->Impose_Rel_Coord(coord);
    }

    void FrNode_::SetLocalPosition(double x, double y, double z) {
        SetLocalPosition(Position(x, y, z));
    }

    void FrNode_::SetLocalQuaternion(const FrUnitQuaternion_ &quaternion) {
        auto coord = chrono::ChCoordsys<double>(chrono::VNULL, internal::Fr2ChQuaternion(quaternion));
        m_chronoMarker->Impose_Rel_Coord(coord);
    }

    void FrNode_::SetLocalRotation(const FrRotation_ &rotation) {
        SetLocalQuaternion(rotation.GetQuaternion());
    }

    void FrNode_::SetLocalFrame(const FrFrame_ &frame) {
        Position localPositionFromCOG = frame.GetPosition(NWU) - m_body->GetCOG(NWU);
        auto chCoord = chrono::ChCoordsys<double>(
                internal::Vector3dToChVector(localPositionFromCOG),
                internal::Fr2ChQuaternion(frame.GetQuaternion())
                );
        m_chronoMarker->Impose_Rel_Coord(chCoord);
    }

    Position FrNode_::GetNodePositionInBody(FRAME_CONVENTION fc) const {
        Position NodePositionInBodyFromCOG = internal::ChVectorToVector3d<Position>(m_chronoMarker->GetRest_Coord().pos);
        Position NodePositionInBody = NodePositionInBodyFromCOG + m_body->GetCOG(NWU);
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(NodePositionInBody);
        return  NodePositionInBody;
    }

    // FIXME : le coord interne de ChMarker est local par rapport au corps auquel il est rattache
    Position FrNode_::GetPositionInWorld(FRAME_CONVENTION fc) const {
        auto PositionInWorld = internal::ChVectorToVector3d<Position>(m_chronoMarker->GetAbsCoord().pos);
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(PositionInWorld);
        return PositionInWorld;
    }

    void FrNode_::GetPositionInWorld(Position &position, FRAME_CONVENTION fc) {
        position = GetPositionInWorld(fc);
    }


    Velocity FrNode_::GetVelocityInWorld(FRAME_CONVENTION fc) const {
        Velocity VelocityInWorld = internal::ChVectorToVector3d<Velocity>(m_chronoMarker->GetAbsCoord_dt().pos);
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(VelocityInWorld);
        return VelocityInWorld;
    }

    Velocity FrNode_::GetVelocityInNode(FRAME_CONVENTION fc) const {
        return ProjectVectorInNode<Velocity>(GetVelocityInWorld(fc),fc);
    }

    Acceleration FrNode_::GetAccelerationInWorld(FRAME_CONVENTION fc) const {
        Acceleration AccelerationInWorld = internal::ChVectorToVector3d<Acceleration>(m_chronoMarker->GetAbsCoord_dtdt().pos);
        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(AccelerationInWorld);
        return AccelerationInWorld;
    }

    Acceleration FrNode_::GetAccelerationInNode(FRAME_CONVENTION fc) const {
        return ProjectVectorInNode<Acceleration>(GetAccelerationInWorld(fc),fc);
    }


    void FrNode_::Initialize() {

    }

    void FrNode_::StepFinalize() {

    }




}  // end namespace frydom
