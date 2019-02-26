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

#include <cppfs/fs.h>
#include <cppfs/FileHandle.h>
#include <cppfs/FilePath.h>

#include "FrNode.h"
#include "frydom/core/body/FrBody.h"


namespace frydom {


    namespace internal {

         FrMarker::FrMarker(frydom::FrNode *node) : m_frydomNode(node) {}

    }  // end namespace frydom::internal


    FrNode::FrNode(frydom::FrBody *body) : m_body(body) {
        m_chronoMarker = std::make_shared<internal::FrMarker>(this);
        body->GetChronoBody()->AddMarker(m_chronoMarker);  //Chrono body can be retrieved because this constructor is a friend of FrBody
    }

    FrBody* FrNode::GetBody() {
        return m_body;
    }

    void FrNode::Set(const Position& position, const Direction& e1, const Direction& e2, const Direction& e3, FRAME_CONVENTION fc) {

        mathutils::Matrix33<double> matrix;                 // FIXME : passer un FrRotation plutôt que matrix33
        matrix <<   e1.Getux(), e2.Getux(), e3.Getux(),
                    e1.Getuy(), e2.Getuy(), e3.Getuy(),
                    e1.Getuz(), e2.Getuz(), e3.Getuz();

        FrUnitQuaternion_ quaternion;
        quaternion.Set(matrix, fc);

        SetFrameInBody(FrFrame(position, quaternion, fc));
    }

    FrFrame FrNode::GetFrameInWorld() const {
        return internal::ChFrame2FrFrame(m_chronoMarker->GetAbsFrame());
    }

    FrFrame FrNode::GetFrameInBody() const {
        auto frame = GetFrameWRT_COG_InBody();
//        frame.SetPosition(frame.GetPosition(NWU) + m_body->GetCOG(NWU), NWU);
        frame.TranslateInParent(m_body->GetCOG(NWU), NWU);  // TODO : comparer cette implementation a la ligne precendente...
        return frame;
    }

    FrFrame FrNode::GetFrameWRT_COG_InBody() const {
        return FrFrame(
                internal::ChVectorToVector3d<Position>(m_chronoMarker->GetPos()),
                internal::Ch2FrQuaternion(m_chronoMarker->GetRot()),
                NWU);
    }

    void FrNode::SetFrameInBody(const FrFrame& frameInBody) {
        Position localPosition_WRT_COG = frameInBody.GetPosition(NWU) - m_body->GetCOG(NWU);
        auto chCoord = chrono::ChCoordsys<double>(
                internal::Vector3dToChVector(localPosition_WRT_COG),
                internal::Fr2ChQuaternion(frameInBody.GetQuaternion())
        );
        m_chronoMarker->Impose_Rel_Coord(chCoord);
    }

    void FrNode::SetFrameInWorld(const FrFrame& frameInWorld) {
        auto chCoord = internal::FrFrame2ChCoordsys(frameInWorld);
        m_chronoMarker->Impose_Abs_Coord(chCoord);
    }

    void FrNode::SetPositionInBody(const Position& bodyPosition, FRAME_CONVENTION fc) {
        auto currentFrameInBody = GetFrameInBody();
        currentFrameInBody.SetPosition(bodyPosition, fc);
        SetFrameInBody(currentFrameInBody);
    }

    void FrNode::SetPositionInWorld(const Position& worldPosition, FRAME_CONVENTION fc) {
        auto currentFrameInWorld = GetFrameInWorld();
        currentFrameInWorld.SetPosition(worldPosition, fc);
        SetFrameInWorld(currentFrameInWorld);
    }

    void FrNode::TranslateInBody(const Translation &translationInBody, FRAME_CONVENTION fc) {
        auto currentFrameInBody = GetFrameInBody();
        currentFrameInBody.TranslateInFrame(translationInBody, fc);
        SetFrameInBody(currentFrameInBody);
    }

    void FrNode::TranslateInBody(const Direction& directionBody, double distance, FRAME_CONVENTION fc) {
        auto tmpDirection = directionBody;
        tmpDirection.Normalize();
        TranslateInBody(distance * tmpDirection, fc);
    }

    void FrNode::TranslateInBody(double x, double y, double z, FRAME_CONVENTION fc) {
        TranslateInBody(Translation(x, y, z), fc);
    }

    void FrNode::TranslateInWorld(const Translation &translationInWorld, FRAME_CONVENTION fc) {
        auto currentFrameInWorld = GetFrameInWorld();
        currentFrameInWorld.TranslateInParent(translationInWorld, fc);
        SetFrameInWorld(currentFrameInWorld);
    }

    void FrNode::TranslateInWorld(const Direction& directionWorld, double distance, FRAME_CONVENTION fc) {
        auto tmpDirection = directionWorld;
        tmpDirection.Normalize();
        TranslateInWorld(distance * tmpDirection, fc);
    }

    void FrNode::TranslateInWorld(double x, double y, double z, FRAME_CONVENTION fc) {
        TranslateInWorld(Translation(x, y, z), fc);
    }

    void FrNode::SetOrientationInBody(const FrRotation& rotation) {
        SetOrientationInBody(rotation.GetQuaternion());
    }

    void FrNode::SetOrientationInBody(const FrUnitQuaternion_& quaternion) {
        auto currentFrameInBody = GetFrameInBody();
        currentFrameInBody.SetRotation(quaternion);
        SetFrameInBody(currentFrameInBody);
    }

    void FrNode::RotateInBody(const FrRotation& rotation) {
        RotateInBody(rotation.GetQuaternion());
    }

    void FrNode::RotateInBody(const FrUnitQuaternion_& quaternion) {
        auto currentFrameInBody = GetFrameInBody();
        currentFrameInBody.RotateInFrame(quaternion);
        SetFrameInBody(currentFrameInBody);
    }

    void FrNode::RotateInWorld(const FrRotation& rotation) {
        RotateInWorld(rotation.GetQuaternion());
    }

    void FrNode::RotateInWorld(const FrUnitQuaternion_& quaternion) {
        auto currentFrameInWorld = GetFrameInWorld();
        currentFrameInWorld.RotateInFrame(quaternion);
        SetFrameInWorld(currentFrameInWorld);
    }

    void FrNode::RotateAroundXInBody(double angleRad, FRAME_CONVENTION fc) {
        FrUnitQuaternion_ quaternion(Direction(1, 0, 0), angleRad, fc);
        RotateInBody(quaternion);
    }

    void FrNode::RotateAroundYInBody(double angleRad, FRAME_CONVENTION fc) {
        FrUnitQuaternion_ quaternion(Direction(0, 1, 0), angleRad, fc);
        RotateInBody(quaternion);
    }

    void FrNode::RotateAroundZInBody(double angleRad, FRAME_CONVENTION fc) {
        FrUnitQuaternion_ quaternion(Direction(0, 0, 1), angleRad, fc);
        RotateInBody(quaternion);
    }

    void FrNode::RotateAroundXInWorld(double angleRad, FRAME_CONVENTION fc) {
        FrUnitQuaternion_ quaternion(Direction(1, 0, 0), angleRad, fc);
        RotateInWorld(quaternion);
    }

    void FrNode::RotateAroundYInWorld(double angleRad, FRAME_CONVENTION fc) {
        FrUnitQuaternion_ quaternion(Direction(0, 1, 0), angleRad, fc);
        RotateInWorld(quaternion);
    }

    void FrNode::RotateAroundZInWorld(double angleRad, FRAME_CONVENTION fc) {
        FrUnitQuaternion_ quaternion(Direction(0, 0, 1), angleRad, fc);
        RotateInWorld(quaternion);
    }

    Position FrNode::GetNodePositionInBody(FRAME_CONVENTION fc) const {
        return GetFrameInBody().GetPosition(fc);
    }


    Position FrNode::GetPositionInWorld(FRAME_CONVENTION fc) const {
        return GetFrameInWorld().GetPosition(fc);
    }

    void FrNode::GetPositionInWorld(Position &position, FRAME_CONVENTION fc) {
        position = GetPositionInWorld(fc);
    }

    Velocity FrNode::GetVelocityInWorld(FRAME_CONVENTION fc) const {
        Velocity VelocityInWorld = internal::ChVectorToVector3d<Velocity>(m_chronoMarker->GetAbsCoord_dt().pos);
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(VelocityInWorld);
        return VelocityInWorld;
    }

    Velocity FrNode::GetVelocityInNode(FRAME_CONVENTION fc) const {
        return ProjectVectorInNode<Velocity>(GetVelocityInWorld(fc),fc);
    }

    Acceleration FrNode::GetAccelerationInWorld(FRAME_CONVENTION fc) const {
        Acceleration AccelerationInWorld = internal::ChVectorToVector3d<Acceleration>(m_chronoMarker->GetAbsCoord_dtdt().pos);
        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(AccelerationInWorld);
        return AccelerationInWorld;
    }

    Acceleration FrNode::GetAccelerationInNode(FRAME_CONVENTION fc) const {
        return ProjectVectorInNode<Acceleration>(GetAccelerationInWorld(fc),fc);
    }

    void FrNode::Initialize() {
        m_chronoMarker->UpdateState();

        InitializeLog();
    }

    void FrNode::StepFinalize() {

//        m_nodeMessage.Serialize();
//        m_nodeMessage.Send();

    }

    void FrNode::InitializeLog(){

//        cppfs::FilePath bodyPath = m_body->GetFilePath();
//
////        cppfs::FilePath nodeLogPath = bodyPath.resolve(fmt::format("Node_{}.csv",GetUUID()));
//
//        // Set the path of the node log
////        SetFilePath(nodeLogPath.path());
//
//        // Initializing message
//        if (m_nodeMessage.GetName().empty()) {
//            m_nodeMessage.SetNameAndDescription(
//                    nodeLogPath.path(),
//                    "Message of a body");
//        }
//
//        // Add a serializer
//        m_nodeMessage.AddCSVSerializer();
//
//        // Add the fields
//        std::function<double ()> GetTime = [this] () {
//            return m_chronoMarker->GetChTime();
//        };
////        m_nodeMessage.AddField<double>("time", "s", "Current time of the simulation", &GetTime);
//
//
//        // Init the message
//        m_nodeMessage.Initialize();
//        m_nodeMessage.Send();


    }

}  // end namespace frydom
