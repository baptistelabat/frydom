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

//    FrNode_::FrNode_(FrBody_ *body, const Position &position, FRAME_CONVENTION fc) : FrNode_(body) {
//        _SetLocalFrame(FrFrame_(position, FrUnitQuaternion_(), fc));
//    }
//
//    FrNode_::FrNode_(FrBody_ *body, const Position &position, const FrRotation_ &rotation, FRAME_CONVENTION fc) : FrNode_(body) {
//        _SetLocalFrame(FrFrame_(position, rotation, fc));
//    }
//
//    FrNode_::FrNode_(FrBody_ *body, const Position &position, const FrUnitQuaternion_ &quaternion, FRAME_CONVENTION fc) : FrNode_(body) {
//        _SetLocalFrame(FrFrame_(position, quaternion, fc));
//    }
//
//    FrNode_::FrNode_(FrBody_ *body, const FrFrame_ &frame) : FrNode_(body) {
//        _SetLocalFrame(frame);
//    }

    FrBody_* FrNode_::GetBody() {
        return m_body;
    }

    void FrNode_::Set(const Position& position, const Direction& e1, const Direction& e2, const Direction& e3, FRAME_CONVENTION fc) {

        mathutils::Matrix33<double> matrix;                 // FIXME : passer un FrRotation plutôt que matrix33
        matrix <<   e1.Getux(), e2.Getux(), e3.Getux(),
                    e1.Getuy(), e2.Getuy(), e3.Getuy(),
                    e1.Getuz(), e2.Getuz(), e3.Getuz();

        FrUnitQuaternion_ quaternion;
        quaternion.Set(matrix, fc);

        SetFrameInBody(FrFrame_(position, quaternion, fc));
    }

    FrFrame_ FrNode_::GetFrameInWorld() const {
        return internal::Ch2FrFrame(m_chronoMarker->GetAbsFrame());
    }

    FrFrame_ FrNode_::GetFrameInBody() const {
        Position pos = m_body->GetCOG(NWU) + internal::ChVectorToVector3d<Position>(m_chronoMarker->GetPos()); // pos wrt body reference frame, not COG..
        auto quat = internal::Ch2FrQuaternion(m_chronoMarker->GetRot());
        return FrFrame_(pos, quat, NWU);
    }

    void FrNode_::SetFrameInBody(const FrFrame_& frameInBody) {
        Position localPosition_WRT_COG = frameInBody.GetPosition(NWU) - m_body->GetCOG(NWU);
        auto chCoord = chrono::ChCoordsys<double>(
                internal::Vector3dToChVector(localPosition_WRT_COG),
                internal::Fr2ChQuaternion(frameInBody.GetQuaternion())
        );
        m_chronoMarker->Impose_Rel_Coord(chCoord);
    }

    void FrNode_::SetFrameInWorld(const FrFrame_& frameInWorld) {
        auto chCoord = internal::Fr2ChCoordsys(frameInWorld);
        m_chronoMarker->Impose_Abs_Coord(chCoord);
    }

    void FrNode_::SetPositionInBody(const Position& bodyPosition, FRAME_CONVENTION fc) {
        auto currentFrameInBody = GetFrameInBody();
        currentFrameInBody.SetPosition(bodyPosition, fc);
        SetFrameInBody(currentFrameInBody);
    }

    void FrNode_::SetPositionInWorld(const Position& worldPosition, FRAME_CONVENTION fc) {
        auto currentFrameInWorld = GetFrameInWorld();
        currentFrameInWorld.SetPosition(worldPosition, fc);
        SetFrameInWorld(currentFrameInWorld);
    }

    void FrNode_::TranslateInBody(const Translation &translationInBody, FRAME_CONVENTION fc) {
        auto currentFrameInBody = GetFrameInBody();
        currentFrameInBody.TranslateInFrame(translationInBody, fc);
        SetFrameInBody(currentFrameInBody);
    }

    void FrNode_::TranslateInBody(const Direction& directionBody, double distance, FRAME_CONVENTION fc) {
        auto tmpDirection = directionBody;
        tmpDirection.Normalize();
        TranslateInBody(distance * tmpDirection, fc);
    }

    void FrNode_::TranslateInBody(double x, double y, double z, FRAME_CONVENTION fc) {
        TranslateInBody(Translation(x, y, z), fc);
    }

    void FrNode_::TranslateInWorld(const Translation &translationInWorld, FRAME_CONVENTION fc) {
        auto currentFrameInWorld = GetFrameInWorld();
        currentFrameInWorld.TranslateInParent(translationInWorld, fc);
        SetFrameInWorld(currentFrameInWorld);
    }

    void FrNode_::TranslateInWorld(const Direction& directionWorld, double distance, FRAME_CONVENTION fc) {
        auto tmpDirection = directionWorld;
        tmpDirection.Normalize();
        TranslateInWorld(distance * tmpDirection, fc);
    }

    void FrNode_::TranslateInWorld(double x, double y, double z, FRAME_CONVENTION fc) {
        TranslateInWorld(Translation(x, y, z), fc);
    }

    void FrNode_::SetOrientationInBody(const FrRotation_& rotation) {
        SetOrientationInBody(rotation.GetQuaternion());
    }

    void FrNode_::SetOrientationInBody(const FrUnitQuaternion_& quaternion) {
        auto currentFrameInBody = GetFrameInBody();
        currentFrameInBody.SetRotation(quaternion);
        SetFrameInBody(currentFrameInBody);
    }

    void FrNode_::RotateInBody(const FrRotation_& rotation) {
        RotateInBody(rotation.GetQuaternion());
    }

    void FrNode_::RotateInBody(const FrUnitQuaternion_& quaternion) {
        auto currentFrameInBody = GetFrameInBody();
        currentFrameInBody.RotateInFrame(quaternion);
        SetFrameInBody(currentFrameInBody);
    }

    void FrNode_::RotateInWorld(const FrRotation_& rotation) {
        RotateInWorld(rotation.GetQuaternion());
    }

    void FrNode_::RotateInWorld(const FrUnitQuaternion_& quaternion) {
        auto currentFrameInWorld = GetFrameInWorld();
        currentFrameInWorld.RotateInFrame(quaternion);
        SetFrameInWorld(currentFrameInWorld);
    }

    void FrNode_::RotateAroundXInBody(double angleRad, FRAME_CONVENTION fc) {
        FrUnitQuaternion_ quaternion(Direction(1, 0, 0), angleRad, fc);
        RotateInBody(quaternion);
    }

    void FrNode_::RotateAroundYInBody(double angleRad, FRAME_CONVENTION fc) {
        FrUnitQuaternion_ quaternion(Direction(0, 1, 0), angleRad, fc);
        RotateInBody(quaternion);
    }

    void FrNode_::RotateAroundZInBody(double angleRad, FRAME_CONVENTION fc) {
        FrUnitQuaternion_ quaternion(Direction(0, 0, 1), angleRad, fc);
        RotateInBody(quaternion);
    }

    void FrNode_::RotateAroundXInWorld(double angleRad, FRAME_CONVENTION fc) {
        FrUnitQuaternion_ quaternion(Direction(1, 0, 0), angleRad, fc);
        RotateInWorld(quaternion);
    }

    void FrNode_::RotateAroundYInWorld(double angleRad, FRAME_CONVENTION fc) {
        FrUnitQuaternion_ quaternion(Direction(0, 1, 0), angleRad, fc);
        RotateInWorld(quaternion);
    }

    void FrNode_::RotateAroundZInWorld(double angleRad, FRAME_CONVENTION fc) {
        FrUnitQuaternion_ quaternion(Direction(0, 0, 1), angleRad, fc);
        RotateInWorld(quaternion);
    }



//    void FrNode_::SetLocalPosition(const Position &position) {
//        Position localPositionFromCOG = position - m_body->GetCOG(NWU);
//        auto coord = chrono::ChCoordsys<double>(chrono::ChVector<double>(internal::Vector3dToChVector(localPositionFromCOG)));
//        m_chronoMarker->Impose_Rel_Coord(coord);
//    }
//
//    void FrNode_::SetLocalPosition(double x, double y, double z) {
//        SetLocalPosition(Position(x, y, z));
//    }
//
//    void FrNode_::SetWorldPosition(const Position& position) {
//        auto coord = chrono::ChCoordsys<double>(
//                internal::Vector3dToChVector(position),
//                m_chronoMarker->GetAbsFrame().GetRot()  // Retrieving the current node orientation wrt world
//                );
//        m_chronoMarker->Impose_Abs_Coord(coord);
//    }
//
//    void FrNode_::SetLocalQuaternion(const FrUnitQuaternion_ &quaternion) {
//        auto coord = chrono::ChCoordsys<double>(chrono::VNULL, internal::Fr2ChQuaternion(quaternion));
//        m_chronoMarker->Impose_Rel_Coord(coord);
//    }
//
//    void FrNode_::SetLocalRotation(const FrRotation_ &rotation) {
//        SetLocalQuaternion(rotation.GetQuaternion());
//    }






//    void FrNode_::_SetLocalFrame(const FrFrame_ &frame) {
//        Position localPositionFromCOG = frame.GetPosition(NWU) - m_body->GetCOG(NWU);
//        auto chCoord = chrono::ChCoordsys<double>(
//                internal::Vector3dToChVector(localPositionFromCOG),
//                internal::Fr2ChQuaternion(frame.GetQuaternion())
//                );
//        m_chronoMarker->Impose_Rel_Coord(chCoord);
//    }

//    void FrNode_::_SetWorldFrame(const frydom::FrFrame_ &frame) {
//        auto chCoord = internal::Fr2ChCoordsys(frame);
//        m_chronoMarker->Impose_Abs_Coord(chCoord);
//    }


    Position FrNode_::GetNodePositionInBody(FRAME_CONVENTION fc) const {
//        Position NodePositionInBodyFromCOG = internal::ChVectorToVector3d<Position>(m_chronoMarker->GetRest_Coord().pos);
//        Position NodePositionInBody = NodePositionInBodyFromCOG + m_body->GetCOG(NWU);
//        if (IsNED(fc)) internal::SwapFrameConvention<Position>(NodePositionInBody);
//        return  NodePositionInBody;
    }

    // FIXME : le coord interne de ChMarker est local par rapport au corps auquel il est rattache
    Position FrNode_::GetPositionInWorld(FRAME_CONVENTION fc) const {
//        auto PositionInWorld = internal::ChVectorToVector3d<Position>(m_chronoMarker->GetAbsCoord().pos);
//        if (IsNED(fc)) internal::SwapFrameConvention<Position>(PositionInWorld);
//        return PositionInWorld;
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
