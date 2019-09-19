//
// Created by lletourn on 22/05/19.
//

#include "FrConstraint.h"

#include "FrCGeometrical.h"
#include "frydom/core/common/FrNode.h"
#include "frydom/core/common/FrFrame.h"

namespace frydom {

    template<typename OffshoreSystemType>
    FrConstraint<OffshoreSystemType>::FrConstraint(const std::shared_ptr<FrNode<OffshoreSystemType>> &node1,
                                                   const std::shared_ptr<FrNode<OffshoreSystemType>> &node2,
                                                   FrOffshoreSystem<OffshoreSystemType> *system)
        : FrLinkBase<OffshoreSystemType>(node1, node2, system) {
      this->SetLogged(true);
    }

    template<typename OffshoreSystemType>
    FrFrame FrConstraint<OffshoreSystemType>::GetConstraintReferenceFrameInWorld() const {
      chrono::ChFrame<> chFrame(this->m_chronoConstraint->GetLinkAbsoluteCoords());
      return internal::ChFrame2FrFrame(chFrame);
    }

    template<typename OffshoreSystemType>
    FrFrame FrConstraint<OffshoreSystemType>::GetConstraintReferenceFrameInBody1() const {
      chrono::ChFrame<> chFrame(this->m_chronoConstraint->GetLinkRelativeCoords());
      return internal::ChFrame2FrFrame(chFrame);
    }

    template<typename OffshoreSystemType>
    Force FrConstraint<OffshoreSystemType>::GetForceInConstraint(FRAME_CONVENTION fc) const {
      auto force = internal::ChVectorToVector3d<Force>(GetChronoItem_ptr()->Get_react_force());
      if (IsNED(fc)) internal::SwapFrameConvention(force);
      return force;
    }

    template<typename OffshoreSystemType>
    Torque FrConstraint<OffshoreSystemType>::GetTorqueInConstraint(FRAME_CONVENTION fc) const {
      auto torque = internal::ChVectorToVector3d<Torque>(GetChronoItem_ptr()->Get_react_torque());
      if (IsNED(fc)) internal::SwapFrameConvention(torque);
      return torque;
    }

    template<typename OffshoreSystemType>
    Force FrConstraint<OffshoreSystemType>::GetForceInBody1(FRAME_CONVENTION fc) const {
      return GetConstraintReferenceFrameInBody1().ProjectVectorFrameInParent(GetForceInConstraint(fc), fc);
    }

    template<typename OffshoreSystemType>
    Torque FrConstraint<OffshoreSystemType>::GetTorqueInBody1AtCOG(FRAME_CONVENTION fc) const {
      auto force = GetForceInBody1(fc);
      auto torque = GetConstraintReferenceFrameInBody1().ProjectVectorFrameInParent(GetTorqueInConstraint(fc), fc);
      auto pos = GetConstraintReferenceFrameInBody1().GetPosition(fc);
      return torque + pos.cross(force);
    }

    template<typename OffshoreSystemType>
    Force FrConstraint<OffshoreSystemType>::GetForceInWorld(FRAME_CONVENTION fc) const {
      return GetConstraintReferenceFrameInWorld().ProjectVectorFrameInParent(GetForceInConstraint(fc), fc);
//        return m_node1->ProjectVectorInWorld(GetForceInConstraint(fc), fc);
    }

    template<typename OffshoreSystemType>
    Torque FrConstraint<OffshoreSystemType>::GetTorqueInWorldAtConstraint(FRAME_CONVENTION fc) const {
      return GetConstraintReferenceFrameInWorld().ProjectVectorFrameInParent(GetTorqueInConstraint(fc), fc);
//        return m_node1->ProjectVectorInWorld(GetTorqueInConstraint(fc), fc);
    }

    template<typename OffshoreSystemType>
    bool FrConstraint<OffshoreSystemType>::IsDisabled() const {
      return this->m_chronoConstraint->IsDisabled();
    }

    template<typename OffshoreSystemType>
    void FrConstraint<OffshoreSystemType>::SetDisabled(bool disabled) {
      this->m_chronoConstraint->SetDisabled(true);
    }

    template<typename OffshoreSystemType>
    bool FrConstraint<OffshoreSystemType>::IsActive() const {
      return this->m_chronoConstraint->IsActive();
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<chrono::ChLink> FrConstraint<OffshoreSystemType>::GetChronoLink() {
      return this->m_chronoConstraint;
    }

    template<typename OffshoreSystemType>
    chrono::ChLink *FrConstraint<OffshoreSystemType>::GetChronoItem_ptr() const {
      return this->m_chronoConstraint.get();
    }

    template<typename OffshoreSystemType>
    void FrConstraint<OffshoreSystemType>::AddFields() {
      this->m_message->template AddField<double>("time", "s", "Current time of the simulation",
                                                 [this]() { return this->m_system->GetTime(); });

      // Constraint position and orientation
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("ConstraintPositionInWorld", "m",
           fmt::format("Constraint reference frame position, relatively to the world reference frame, in {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetConstraintReferenceFrameInWorld().GetPosition(this->GetLogFrameConvention()); });
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("ConstraintOrientationInWorld", "deg",
           fmt::format("Constraint reference frame orientation, relatively to the world reference frame, in {}",
                       this->GetLogFrameConvention()),
           [this]() {
             double phi, theta, psi;
             GetConstraintReferenceFrameInWorld().GetRotation().GetCardanAngles_DEGREES(phi, theta, psi,
                                                                                        this->GetLogFrameConvention());
             return Position(phi, theta, psi);
           });

      // Constraint reaction force and torque
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("GetForceInWorld", "N",
           fmt::format("Constraint reaction force in world reference frame, in {}", this->GetLogFrameConvention()),
           [this]() { return GetForceInWorld(this->GetLogFrameConvention()); });
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("GetTorqueInWorldAtConstraint", "Nm", fmt::format(
              "Constraint reaction torque at constraint reference frame origin, in world reference frame, in {}",
              this->GetLogFrameConvention()),
           [this]() { return GetTorqueInWorldAtConstraint(this->GetLogFrameConvention()); });

      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("GetForceInBody1", "N",
           fmt::format("Constraint reaction force in first body reference frame, in {}", this->GetLogFrameConvention()),
           [this]() { return GetForceInBody1(this->GetLogFrameConvention()); });
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("GetTorqueInBody1AtCOG", "Nm",
           fmt::format("Constraint reaction torque at COG, in first body reference frame, in {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetTorqueInBody1AtCOG(this->GetLogFrameConvention()); });
    }

    //------------------------------------------------------------------------------------------------------------------
    template<typename OffshoreSystemType>
    FrConstraintParallel<OffshoreSystemType>::FrConstraintParallel(
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis1,
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis2, FrOffshoreSystem<OffshoreSystemType> *system) :
        FrConstraint<OffshoreSystemType>(axis1->GetNode(), axis2->GetNode(), system), m_axis1(axis1), m_axis2(axis2) {
      this->m_chronoConstraint = std::make_shared<chrono::ChLinkMateParallel>();
    }

    template<typename OffshoreSystemType>
    void FrConstraintParallel<OffshoreSystemType>::Initialize() {

      auto chPos1 = internal::Vector3dToChVector(m_axis1->GetOriginInWorld(NWU));
      auto chPos2 = internal::Vector3dToChVector(m_axis2->GetOriginInWorld(NWU));
      auto chDir1 = internal::Vector3dToChVector(m_axis1->GetDirectionInWorld(NWU));
      auto chDir2 = internal::Vector3dToChVector(m_axis2->GetDirectionInWorld(NWU));

      GetChronoItem_ptr()->Initialize(
          this->GetChronoBody2(),
          this->GetChronoBody1(),
          false, chPos2, chPos1, chDir2, chDir1);

    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrConstraintParallel<OffshoreSystemType>>
    make_constraint_parallel(
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis1,
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis2,
        FrOffshoreSystem<OffshoreSystemType> *system) {

      auto constraint = std::make_shared<FrConstraintParallel>(axis1, axis2, system);
      system->AddLink(constraint);

      return constraint;

    }

    //------------------------------------------------------------------------------------------------------------------
    template<typename OffshoreSystemType>
    FrConstraintPerpendicular<OffshoreSystemType>::FrConstraintPerpendicular(
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis1,
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis2,
        FrOffshoreSystem<OffshoreSystemType> *system) :
        FrConstraint<OffshoreSystemType>(axis1->GetNode(), axis2->GetNode(), system), m_axis1(axis1), m_axis2(axis2) {
      this->m_chronoConstraint = std::make_shared<chrono::ChLinkMateOrthogonal>();
    }

    template<typename OffshoreSystemType>
    void FrConstraintPerpendicular<OffshoreSystemType>::Initialize() {

      auto chPos1 = internal::Vector3dToChVector(m_axis1->GetOriginInWorld(NWU));
      auto chDir1 = internal::Vector3dToChVector(m_axis1->GetDirectionInWorld(NWU));
      auto chPos2 = internal::Vector3dToChVector(m_axis2->GetOriginInWorld(NWU));
      auto chDir2 = internal::Vector3dToChVector(m_axis2->GetDirectionInWorld(NWU));

      GetChronoItem_ptr()->Initialize(this->GetChronoBody2(), this->GetChronoBody1(), false, chPos2, chPos1, chDir2,
                                      chDir1);

    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrConstraintPerpendicular<OffshoreSystemType>>
    make_constraint_perpendicular(
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis1,
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis2,
        FrOffshoreSystem<OffshoreSystemType> *system) {

      auto constraint = std::make_shared<FrConstraintPerpendicular>(axis1, axis2, system);
      system->AddLink(constraint);

      return constraint;
    }

    //------------------------------------------------------------------------------------------------------------------
    template<typename OffshoreSystemType>
    FrConstraintPlaneOnPlane<OffshoreSystemType>::FrConstraintPlaneOnPlane(
        const std::shared_ptr<FrCPlane<OffshoreSystemType>> &plane1,
        const std::shared_ptr<FrCPlane<OffshoreSystemType>> &plane2,
        FrOffshoreSystem<OffshoreSystemType> *system,
        bool flipped,
        double distance) :
        FrConstraint<OffshoreSystemType>(plane1->GetNode(), plane2->GetNode(), system), m_plane1(plane1),
        m_plane2(plane2) {
      this->m_chronoConstraint = std::make_shared<chrono::ChLinkMatePlane>();
      SetFlipped(flipped);
      SetDistance(distance);
    }

    template<typename OffshoreSystemType>
    void FrConstraintPlaneOnPlane<OffshoreSystemType>::Initialize() {

      auto chPos1 = internal::Vector3dToChVector(m_plane1->GetOriginInWorld(NWU));
      auto chDir1 = internal::Vector3dToChVector(m_plane1->GetNormaleInWorld(NWU));
      auto chPos2 = internal::Vector3dToChVector(m_plane2->GetOriginInWorld(NWU));
      auto chDir2 = internal::Vector3dToChVector(m_plane2->GetNormaleInWorld(NWU));

      GetChronoItem_ptr()->Initialize(this->GetChronoBody2(), this->GetChronoBody1(), false, chPos2, chPos1, chDir2,
                                      chDir1);

    }

    template<typename OffshoreSystemType>
    void FrConstraintPlaneOnPlane<OffshoreSystemType>::SetFlipped(bool flip) {
      GetChronoItem_ptr()->SetFlipped(flip);
    }

    template<typename OffshoreSystemType>
    void FrConstraintPlaneOnPlane<OffshoreSystemType>::SetDistance(double distance) {
      GetChronoItem_ptr()->SetSeparation(distance);
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrConstraintPlaneOnPlane<OffshoreSystemType>>
    make_constraint_plane_on_plane(
        const std::shared_ptr<FrCPlane<OffshoreSystemType>> &plane1,
        const std::shared_ptr<FrCPlane<OffshoreSystemType>> &plane2,
        FrOffshoreSystem<OffshoreSystemType> *system,
        bool flipped,
        double distance) {

      auto constraint = std::make_shared<FrConstraintPlaneOnPlane>(plane1, plane2, system, flipped, distance);
      system->AddLink(constraint);

      return constraint;
    }

    //------------------------------------------------------------------------------------------------------------------
    template<typename OffshoreSystemType>
    FrConstraintPointOnPlane<OffshoreSystemType>::FrConstraintPointOnPlane(
        const std::shared_ptr<FrCPlane<OffshoreSystemType>> &plane,
        const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point,
        FrOffshoreSystem<OffshoreSystemType> *system,
        double distance):
        FrConstraint<OffshoreSystemType>(plane->GetNode(), point->GetNode(), system), m_plane(plane), m_point(point) {
      this->m_chronoConstraint = std::make_shared<chrono::ChLinkMateXdistance>();
      SetDistance(distance);
    }

    template<typename OffshoreSystemType>
    void FrConstraintPointOnPlane<OffshoreSystemType>::Initialize() {

      auto chPos2 = internal::Vector3dToChVector(m_point->GetPositionInWorld(NWU));
      auto chPos1 = internal::Vector3dToChVector(m_plane->GetOriginInWorld(NWU));
      auto chDir1 = internal::Vector3dToChVector(m_plane->GetNormaleInWorld(NWU));

      GetChronoItem_ptr()->Initialize(this->GetChronoBody2(), this->GetChronoBody1(), false, chPos2, chPos1, chDir1);
    }

    template<typename OffshoreSystemType>
    void FrConstraintPointOnPlane<OffshoreSystemType>::SetDistance(double distance) {
      GetChronoItem_ptr()->SetDistance(distance);
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrConstraintPointOnPlane<OffshoreSystemType>>
    make_constraint_point_on_plane(
        const std::shared_ptr<FrCPlane<OffshoreSystemType>> &plane,
        const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point,
        FrOffshoreSystem<OffshoreSystemType> *system,
        double distance) {

      auto constraint = std::make_shared<FrConstraintPointOnPlane>(plane, point, system, distance);
      system->AddLink(constraint);

      return constraint;

    }

    //------------------------------------------------------------------------------------------------------------------
    template<typename OffshoreSystemType>
    FrConstraintPointOnLine<OffshoreSystemType>::FrConstraintPointOnLine(
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &line,
        const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point,
        FrOffshoreSystem<OffshoreSystemType> *system,
        double distance):
        FrConstraint<OffshoreSystemType>(line->GetNode(), point->GetNode(), system), m_point(point), m_axis(line) {
      this->m_chronoConstraint = std::make_shared<chrono::ChLinkLockPointLine>();
    }

    template<typename OffshoreSystemType>
    void FrConstraintPointOnLine<OffshoreSystemType>::Initialize() {

//        auto chPos2 = internal::Vector3dToChVector(m_point->GetPositionInWorld(NWU));
//        auto chPos1 = internal::Vector3dToChVector(m_axis->GetOriginInWorld(NWU));
//        auto chDir1 = internal::Vector3dToChVector(m_axis->GetDirectionInWorld(NWU));

      auto axisFrame = m_axis->GetNode()->GetFrameInWorld();
      if (m_axis->GetLabel() == YAXIS) axisFrame.RotZ_DEGREES(90, NWU, true);
      if (m_axis->GetLabel() == ZAXIS) axisFrame.RotY_DEGREES(90, NWU, true);


      auto chCoordSys1 = internal::FrFrame2ChCoordsys(axisFrame);
      auto chCoordSys2 = internal::FrFrame2ChCoordsys(m_point->GetNode()->GetFrameInWorld());

      GetChronoItem_ptr()->Initialize(this->GetChronoBody2(), this->GetChronoBody1(), false, chCoordSys2, chCoordSys1);

    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrConstraintPointOnLine<OffshoreSystemType>>
    make_constraint_point_on_line(
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &line,
        const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point,
        FrOffshoreSystem<OffshoreSystemType> *system) {

      auto constraint = std::make_shared<FrConstraintPointOnLine>(line, point, system);
      system->AddLink(constraint);

      return constraint;

    }


    //------------------------------------------------------------------------------------------------------------------
    template<typename OffshoreSystemType>
    FrConstraintDistanceToAxis<OffshoreSystemType>::FrConstraintDistanceToAxis(
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis,
        const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point,
        FrOffshoreSystem<OffshoreSystemType> *system,
        bool autoDistance,
        double distance) :
        FrConstraint<OffshoreSystemType>(axis->GetNode(), point->GetNode(), system), m_point(point), m_axis(axis),
        m_autoDistance(autoDistance) {
      this->m_chronoConstraint = std::make_shared<chrono::ChLinkRevoluteSpherical>();
      SetDistance(distance);
    }

    template<typename OffshoreSystemType>
    void FrConstraintDistanceToAxis<OffshoreSystemType>::Initialize() {

      auto chPos2 = internal::Vector3dToChVector(m_point->GetPositionInWorld(NWU));
      auto chPos1 = internal::Vector3dToChVector(m_axis->GetOriginInWorld(NWU));
      auto chDir1 = internal::Vector3dToChVector(m_axis->GetDirectionInWorld(NWU));

      GetChronoItem_ptr()->Initialize(this->GetChronoBody1(), this->GetChronoBody2(), false, chPos1, chDir1, chPos2,
                                      m_autoDistance,
                                      GetDistance());

    }

    template<typename OffshoreSystemType>
    void FrConstraintDistanceToAxis<OffshoreSystemType>::SetDistance(double distance) {
      m_distance = distance;
    }

    template<typename OffshoreSystemType>
    double FrConstraintDistanceToAxis<OffshoreSystemType>::GetDistance() const {
      return m_distance;
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrConstraintDistanceToAxis<OffshoreSystemType>>
    make_constraint_distance_to_axis(
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis,
        const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point,
        FrOffshoreSystem<OffshoreSystemType> *system,
        bool autoDistance,
        double distance) {

      auto constraint = std::make_shared<FrConstraintDistanceToAxis>(axis, point, system, autoDistance, distance);
      system->AddLink(constraint);

      return constraint;

    }

    //------------------------------------------------------------------------------------------------------------------
    template<typename OffshoreSystemType>
    FrConstraintDistanceBetweenPoints<OffshoreSystemType>::FrConstraintDistanceBetweenPoints(
        const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point1,
        const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point2,
        FrOffshoreSystem<OffshoreSystemType> *system,
        bool autoDistance,
        double distance) :
        FrConstraint<OffshoreSystemType>(point1->GetNode(), point2->GetNode(), system),
        m_point1(point1), m_point2(point2), m_autoDistance(autoDistance) {
      this->m_chronoConstraint = std::make_shared<chrono::ChLinkDistance>();
      SetDistance(distance);
    }

    template<typename OffshoreSystemType>
    void FrConstraintDistanceBetweenPoints<OffshoreSystemType>::SetDistance(double distance) {
      m_distance = distance;
    }

    template<typename OffshoreSystemType>
    double FrConstraintDistanceBetweenPoints<OffshoreSystemType>::GetDistance() const {
      return m_distance;
    }

    template<typename OffshoreSystemType>
    void FrConstraintDistanceBetweenPoints<OffshoreSystemType>::Initialize() {

      auto chPos1 = internal::Vector3dToChVector(m_point1->GetPositionInWorld(NWU));
      auto chPos2 = internal::Vector3dToChVector(m_point2->GetPositionInWorld(NWU));

      GetChronoItem_ptr()->Initialize(this->GetChronoBody1(), this->GetChronoBody2(), false, chPos1, chPos2,
                                      m_autoDistance,
                                      GetDistance());
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrConstraintDistanceBetweenPoints<OffshoreSystemType>>
    make_constraint_distance_between_points(const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point1,
                                            const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point2,
                                            FrOffshoreSystem<OffshoreSystemType> *system,
                                            bool autoDistance,
                                            double distance) {

      auto constraint = std::make_shared<FrConstraintDistanceBetweenPoints>(point1, point2, system, autoDistance,
                                                                            distance);
      system->AddLink(constraint);

      return constraint;

    }
}
