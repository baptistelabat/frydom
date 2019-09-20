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
#include "frydom/asset/FrNodeAsset.h"
#include "frydom/core/body/FrBody.h"

#include "frydom/utils/FrSerializerFactory.h"


namespace frydom {


    namespace internal {

        template<typename OffshoreSystemType>
        FrMarker<OffshoreSystemType>::FrMarker(frydom::FrNode<OffshoreSystemType> *node) : m_frydomNode(node) {}

    }  // end namespace frydom::internal

    template<typename OffshoreSystemType>
    FrNode<OffshoreSystemType>::FrNode(frydom::FrBody<OffshoreSystemType> *body) : m_body(body), m_showAsset(false) {
      m_chronoMarker = std::make_shared<internal::FrMarker>(this);
      body->GetChronoBody()->AddMarker(
          m_chronoMarker);  //Chrono body can be retrieved because this constructor is a friend of FrBody
    }

    template<typename OffshoreSystemType>
    FrBody<OffshoreSystemType> *FrNode<OffshoreSystemType>::GetBody() {
      return m_body;
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::Set(const Position &position, const Direction &e1, const Direction &e2, const Direction &e3,
                     FRAME_CONVENTION fc) {

      mathutils::Matrix33<double> matrix;                 // FIXME : passer un FrRotation plutôt que matrix33
      matrix << e1.Getux(), e2.Getux(), e3.Getux(),
          e1.Getuy(), e2.Getuy(), e3.Getuy(),
          e1.Getuz(), e2.Getuz(), e3.Getuz();

      FrUnitQuaternion quaternion;
      quaternion.Set(matrix, fc);

      SetFrameInBody(FrFrame(position, quaternion, fc));
    }

    template<typename OffshoreSystemType>
    FrFrame FrNode<OffshoreSystemType>::GetFrameInWorld() const {
      return internal::ChFrame2FrFrame(m_chronoMarker->GetAbsFrame());
    }

    template<typename OffshoreSystemType>
    FrFrame FrNode<OffshoreSystemType>::GetFrameInBody() const {
      auto frame = GetFrameWRT_COG_InBody();
//        frame.SetPosition(frame.GetPosition(NWU) + m_body->GetCOG(NWU), NWU);
      frame.TranslateInParent(m_body->GetCOG(NWU),
                              NWU);  // TODO : comparer cette implementation a la ligne precendente...
      return frame;
    }

    template<typename OffshoreSystemType>
    FrFrame FrNode<OffshoreSystemType>::GetFrameWRT_COG_InBody() const {
      return FrFrame(
          internal::ChVectorToVector3d<Position>(m_chronoMarker->GetPos()),
          internal::Ch2FrQuaternion(m_chronoMarker->GetRot()),
          NWU);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::SetFrameInBody(const FrFrame &frameInBody) {
      Position localPosition_WRT_COG = frameInBody.GetPosition(NWU) - m_body->GetCOG(NWU);
      auto chCoord = chrono::ChCoordsys<double>(
          internal::Vector3dToChVector(localPosition_WRT_COG),
          internal::Fr2ChQuaternion(frameInBody.GetQuaternion())
      );
      m_chronoMarker->Impose_Rel_Coord(chCoord);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::SetFrameInWorld(const FrFrame &frameInWorld) {
      auto chCoord = internal::FrFrame2ChCoordsys(frameInWorld);
      m_chronoMarker->Impose_Abs_Coord(chCoord);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::SetPositionInBody(const Position &bodyPosition, FRAME_CONVENTION fc) {
      auto currentFrameInBody = GetFrameInBody();
      currentFrameInBody.SetPosition(bodyPosition, fc);
      SetFrameInBody(currentFrameInBody);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::SetPositionInWorld(const Position &worldPosition, FRAME_CONVENTION fc) {
      auto currentFrameInWorld = GetFrameInWorld();
      currentFrameInWorld.SetPosition(worldPosition, fc);
      SetFrameInWorld(currentFrameInWorld);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::TranslateInBody(const Translation &translationInBody, FRAME_CONVENTION fc) {
      auto currentFrameInBody = GetFrameInBody();
      currentFrameInBody.TranslateInFrame(translationInBody, fc);
      SetFrameInBody(currentFrameInBody);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::TranslateInBody(const Direction &directionBody, double distance, FRAME_CONVENTION fc) {
      auto tmpDirection = directionBody;
      tmpDirection.Normalize();
      TranslateInBody(distance * tmpDirection, fc);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::TranslateInBody(double x, double y, double z, FRAME_CONVENTION fc) {
      TranslateInBody(Translation(x, y, z), fc);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::TranslateInWorld(const Translation &translationInWorld, FRAME_CONVENTION fc) {
      auto currentFrameInWorld = GetFrameInWorld();
      currentFrameInWorld.TranslateInParent(translationInWorld, fc);
      SetFrameInWorld(currentFrameInWorld);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::TranslateInWorld(const Direction &directionWorld, double distance, FRAME_CONVENTION fc) {
      auto tmpDirection = directionWorld;
      tmpDirection.Normalize();
      TranslateInWorld(distance * tmpDirection, fc);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::TranslateInWorld(double x, double y, double z, FRAME_CONVENTION fc) {
      TranslateInWorld(Translation(x, y, z), fc);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::SetOrientationInBody(const FrRotation &rotation) {
      SetOrientationInBody(rotation.GetQuaternion());
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::SetOrientationInBody(const FrUnitQuaternion &quaternion) {
      auto currentFrameInBody = GetFrameInBody();
      currentFrameInBody.SetRotation(quaternion);
      SetFrameInBody(currentFrameInBody);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::RotateInBody(const FrRotation &rotation) {
      RotateInBody(rotation.GetQuaternion());
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::RotateInBody(const FrUnitQuaternion &quaternion) {
      auto currentFrameInBody = GetFrameInBody();
      currentFrameInBody.RotateInFrame(quaternion);
      SetFrameInBody(currentFrameInBody);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::RotateInWorld(const FrRotation &rotation) {
      RotateInWorld(rotation.GetQuaternion());
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::RotateInWorld(const FrUnitQuaternion &quaternion) {
      auto currentFrameInWorld = GetFrameInWorld();
      currentFrameInWorld.RotateInFrame(quaternion);
      SetFrameInWorld(currentFrameInWorld);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::RotateAroundXInBody(double angleRad, FRAME_CONVENTION fc) {
      FrUnitQuaternion quaternion(Direction(1, 0, 0), angleRad, fc);
      RotateInBody(quaternion);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::RotateAroundYInBody(double angleRad, FRAME_CONVENTION fc) {
      FrUnitQuaternion quaternion(Direction(0, 1, 0), angleRad, fc);
      RotateInBody(quaternion);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::RotateAroundZInBody(double angleRad, FRAME_CONVENTION fc) {
      FrUnitQuaternion quaternion(Direction(0, 0, 1), angleRad, fc);
      RotateInBody(quaternion);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::RotateAroundXInWorld(double angleRad, FRAME_CONVENTION fc) {
      FrUnitQuaternion quaternion(Direction(1, 0, 0), angleRad, fc);
      RotateInWorld(quaternion);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::RotateAroundYInWorld(double angleRad, FRAME_CONVENTION fc) {
      FrUnitQuaternion quaternion(Direction(0, 1, 0), angleRad, fc);
      RotateInWorld(quaternion);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::RotateAroundZInWorld(double angleRad, FRAME_CONVENTION fc) {
      FrUnitQuaternion quaternion(Direction(0, 0, 1), angleRad, fc);
      RotateInWorld(quaternion);
    }

    template<typename OffshoreSystemType>
    Position FrNode<OffshoreSystemType>::GetNodePositionInBody(FRAME_CONVENTION fc) const {
      return GetFrameInBody().GetPosition(fc);
    }

    template<typename OffshoreSystemType>
    Position FrNode<OffshoreSystemType>::GetPositionInWorld(FRAME_CONVENTION fc) const {
      return GetFrameInWorld().GetPosition(fc);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::GetPositionInWorld(Position &position, FRAME_CONVENTION fc) {
      position = GetPositionInWorld(fc);
    }

    template<typename OffshoreSystemType>
    Velocity FrNode<OffshoreSystemType>::GetVelocityInWorld(FRAME_CONVENTION fc) const {
      Velocity VelocityInWorld = internal::ChVectorToVector3d<Velocity>(m_chronoMarker->GetAbsCoord_dt().pos);
      if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(VelocityInWorld);
      return VelocityInWorld;
    }

    template<typename OffshoreSystemType>
    Velocity FrNode<OffshoreSystemType>::GetVelocityInNode(FRAME_CONVENTION fc) const {
      return ProjectVectorInNode<Velocity>(GetVelocityInWorld(fc), fc);
    }

    template<typename OffshoreSystemType>
    AngularVelocity FrNode<OffshoreSystemType>::GetAngularVelocityInWorld(FRAME_CONVENTION fc) const {
      AngularVelocity AngularVelocityInWorld = internal::ChVectorToVector3d<AngularVelocity>(
          m_chronoMarker->GetAbsWvel());
      if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(AngularVelocityInWorld);
      return AngularVelocityInWorld;
    }

    template<typename OffshoreSystemType>
    AngularVelocity FrNode<OffshoreSystemType>::GetAngularVelocityInBody(FRAME_CONVENTION fc) const {
      AngularVelocity AngularVelocityInWorld = internal::ChVectorToVector3d<AngularVelocity>(
          m_chronoMarker->GetWvel_par());
      if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(AngularVelocityInWorld);
      return AngularVelocityInWorld;

    }

    template<typename OffshoreSystemType>
    AngularVelocity FrNode<OffshoreSystemType>::GetAngularVelocityInNode(FRAME_CONVENTION fc) const {
      AngularVelocity AngularVelocityInWorld = internal::ChVectorToVector3d<AngularVelocity>(
          m_chronoMarker->GetWvel_loc());
      if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(AngularVelocityInWorld);
      return AngularVelocityInWorld;

    }

    template<typename OffshoreSystemType>
    Acceleration FrNode<OffshoreSystemType>::GetAccelerationInWorld(FRAME_CONVENTION fc) const {
      Acceleration AccelerationInWorld = internal::ChVectorToVector3d<Acceleration>(
          m_chronoMarker->GetAbsCoord_dtdt().pos);
      if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(AccelerationInWorld);
      return AccelerationInWorld;
    }

    template<typename OffshoreSystemType>
    Acceleration FrNode<OffshoreSystemType>::GetAccelerationInNode(FRAME_CONVENTION fc) const {
      return ProjectVectorInNode<Acceleration>(GetAccelerationInWorld(fc), fc);
    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::Initialize() {

      m_chronoMarker->UpdateState();

      if (m_showAsset) {
        m_asset->Initialize();
        m_body->AddAsset(m_asset);
      }

    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::AddFields() {

      if (this->IsLogged()) {

        // Add the fields to be logged here
        this->m_message->template AddField<double>("time", "s", "Current time of the simulation",
                                    [this]() { return m_chronoMarker->GetChTime(); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("PositionInWorld", "m",
             fmt::format("Node position in world reference frame in {}", this->GetLogFrameConvention()),
             [this]() { return GetPositionInWorld(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("VelocityInWorld", "m/s",
             fmt::format("Node velocity in world reference frame in {}", this->GetLogFrameConvention()),
             [this]() { return GetVelocityInWorld(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("AccelerationInWorld", "m/s²",
             fmt::format("Node acceleration in world reference frame in {}", this->GetLogFrameConvention()),
             [this]() { return GetAccelerationInWorld(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("NodePositionInBody", "m",
             fmt::format("Node position in body reference frame in {}", this->GetLogFrameConvention()),
             [this]() { return GetNodePositionInBody(this->GetLogFrameConvention()); });

      }

    }

    template<typename OffshoreSystemType>
    void FrNode<OffshoreSystemType>::ShowAsset(bool showAsset) {
      m_showAsset = showAsset;
      if (showAsset) {
        assert(m_asset == nullptr);
        m_asset = std::make_shared<FrNodeAsset>(this);
      }
    }

    template<typename OffshoreSystemType>
    FrNodeAsset<OffshoreSystemType> *FrNode<OffshoreSystemType>::GetAsset() {
      return m_asset.get();
    }

    template<typename OffshoreSystemType>
    std::string FrNode<OffshoreSystemType>::BuildPath(const std::string &rootPath) {

      auto objPath = fmt::format("{}/Nodes", rootPath);

      auto logPath = this->GetPathManager()->BuildPath(objPath, fmt::format("{}_{}.csv", GetTypeName(), this->GetShortenUUID()));

      // Add a serializer
      this->m_message->AddSerializer(FrSerializerFactory<OffshoreSystemType>::instance().Create(this, logPath));

      return objPath;
    }

}  // end namespace frydom
