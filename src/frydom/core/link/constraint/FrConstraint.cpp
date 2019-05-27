//
// Created by lletourn on 22/05/19.
//

#include "FrConstraint.h"

#include "frydom/core/common/FrGeometrical.h"
#include "frydom/core/common/FrNode.h"
#include "frydom/core/common/FrFrame.h"

namespace frydom {


    FrConstraint::FrConstraint(const std::shared_ptr<FrNode> &node1, const std::shared_ptr<FrNode> &node2,
                               FrOffshoreSystem *system) : FrLinkBase(node1, node2, system) {
        SetLogged(true);
    }

    FrFrame FrConstraint::GetLinkReferenceFrameInWorld() const {
        chrono::ChFrame<> chFrame(m_chronoConstraint->GetLinkAbsoluteCoords());
        return internal::ChFrame2FrFrame(chFrame);
    }

    FrFrame FrConstraint::GetLinkReferenceFrameInBody1() const {
        chrono::ChFrame<> chFrame(m_chronoConstraint->GetLinkRelativeCoords());
        return internal::ChFrame2FrFrame(chFrame);
    }

    Force FrConstraint::GetForceInLink(FRAME_CONVENTION fc) const {
        auto force = internal::ChVectorToVector3d<Force>(GetChronoItem_ptr()->Get_react_force());
        if (IsNED(fc)) internal::SwapFrameConvention(force);
        return force;
    }

    Torque FrConstraint::GetTorqueInLink(FRAME_CONVENTION fc) const {
        auto torque = internal::ChVectorToVector3d<Torque>(GetChronoItem_ptr()->Get_react_torque());
        if (IsNED(fc)) internal::SwapFrameConvention(torque);
        return torque;
    }

    Force FrConstraint::GetForceInBody1(FRAME_CONVENTION fc) const {
        return GetLinkReferenceFrameInBody1().ProjectVectorFrameInParent(GetForceInLink(fc), fc);
    }

    Torque FrConstraint::GetTorqueInBody1AtCOG(FRAME_CONVENTION fc) const {
        auto force = GetForceInBody1(fc);
        auto torque = GetLinkReferenceFrameInBody1().ProjectVectorFrameInParent(GetTorqueInLink(fc), fc);
        auto pos = GetLinkReferenceFrameInBody1().GetPosition(fc);
        return torque + pos.cross(force);
    }

    Force FrConstraint::GetForceInWorld(FRAME_CONVENTION fc) const {
        return GetLinkReferenceFrameInWorld().ProjectVectorFrameInParent(GetForceInLink(fc), fc);
//        return m_node1->ProjectVectorInWorld(GetForceInLink(fc), fc);
    }

    Torque FrConstraint::GetTorqueInWorldAtLink(FRAME_CONVENTION fc) const {
        return GetLinkReferenceFrameInWorld().ProjectVectorFrameInParent(GetTorqueInLink(fc), fc);
//        return m_node1->ProjectVectorInWorld(GetTorqueInLink(fc), fc);
    }

    bool FrConstraint::IsDisabled() const {
        return m_chronoConstraint->IsDisabled();
    }

    void FrConstraint::SetDisabled(bool disabled) {
        m_chronoConstraint->SetDisabled(true);
    }

    bool FrConstraint::IsActive() const {
        return m_chronoConstraint->IsActive();
    }

    std::shared_ptr<chrono::ChLink> FrConstraint::GetChronoLink() {
        return m_chronoConstraint;
    }

    chrono::ChLink *FrConstraint::GetChronoItem_ptr() const {
        return m_chronoConstraint.get();
    }

    void FrConstraint::AddFields() {
        m_message->AddField<double>("time", "s", "Current time of the simulation",
                                    [this]() { return m_system->GetTime(); });

        // Constraint position and orientation
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("LinkPositionInWorld","m", fmt::format("Link reference frame position, relatively to the world reference frame, in {}",GetLogFrameConvention()),
                 [this]() {return GetLinkReferenceFrameInWorld().GetPosition(GetLogFrameConvention());});
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("LinkOrientationInWorld","deg", fmt::format("Link reference frame orientation, relatively to the world reference frame, in {}",GetLogFrameConvention()),
                 [this]() { double phi, theta, psi; GetLinkReferenceFrameInWorld().GetRotation().GetCardanAngles_DEGREES(phi, theta, psi, GetLogFrameConvention()) ;
                    return Position(phi, theta, psi);});

        // Constraint reaction force and torque
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("GetForceInWorld","N", fmt::format("Constraint reaction force in world reference frame, in {}",GetLogFrameConvention()),
                 [this]() {return GetForceInWorld(GetLogFrameConvention());});
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("GetTorqueInWorldAtLink","Nm", fmt::format("Constraint reaction torque at link reference frame origin, in world reference frame, in {}",GetLogFrameConvention()),
                 [this]() {return GetTorqueInWorldAtLink(GetLogFrameConvention());});

        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("GetForceInBody1","N", fmt::format("Constraint reaction force in first body reference frame, in {}",GetLogFrameConvention()),
                 [this]() {return GetForceInBody1(GetLogFrameConvention());});
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("GetTorqueInBody1AtCOG","Nm", fmt::format("Constraint reaction torque at COG, in first body reference frame, in {}",GetLogFrameConvention()),
                 [this]() {return GetTorqueInBody1AtCOG(GetLogFrameConvention());});
    }

    //------------------------------------------------------------------------------------------------------------------

    FrConstraintParallel::FrConstraintParallel(const std::shared_ptr<FrAxis>& axis1, const std::shared_ptr<FrAxis>& axis2, FrOffshoreSystem* system) :
            FrConstraint(axis1->GetNode(), axis2->GetNode(), system), m_axis1(axis1), m_axis2( axis2) {
            m_chronoConstraint = std:: make_shared<chrono::ChLinkMateParallel>();
    }

    void FrConstraintParallel::Initialize() {

        auto chPos1 = internal::Vector3dToChVector(m_axis1->GetOriginInWorld(NWU));
        auto chPos2 = internal::Vector3dToChVector(m_axis2->GetOriginInWorld(NWU));
        auto chDir1 = internal::Vector3dToChVector(m_axis1->GetDirectionInWorld(NWU));
        auto chDir2 = internal::Vector3dToChVector(m_axis2->GetDirectionInWorld(NWU));

        GetChronoItem_ptr()->Initialize(GetChronoBody2(), GetChronoBody1(), false, chPos2, chPos1, chDir2, chDir1);

    }
    std::shared_ptr<FrConstraintParallel>
    make_constraint_parallel(
            const std::shared_ptr<FrAxis>& axis1,
            const std::shared_ptr<FrAxis>& axis2,
            FrOffshoreSystem* system) {

        auto constraint = std::make_shared<FrConstraintParallel>(axis1, axis2, system);
        system->AddLink(constraint);

        return constraint;

    }

    //------------------------------------------------------------------------------------------------------------------

    FrConstraintPerpendicular::FrConstraintPerpendicular(const std::shared_ptr<FrAxis>& axis1, const std::shared_ptr<FrAxis>& axis2, FrOffshoreSystem* system) :
            FrConstraint(axis1->GetNode(), axis2->GetNode(), system), m_axis1(axis1), m_axis2( axis2) {
        m_chronoConstraint = std:: make_shared<chrono::ChLinkMateOrthogonal>();
    }

    void FrConstraintPerpendicular::Initialize() {

        auto chPos1 = internal::Vector3dToChVector(m_axis1->GetOriginInWorld(NWU));
        auto chDir1 = internal::Vector3dToChVector(m_axis1->GetDirectionInWorld(NWU));
        auto chPos2 = internal::Vector3dToChVector(m_axis2->GetOriginInWorld(NWU));
        auto chDir2 = internal::Vector3dToChVector(m_axis2->GetDirectionInWorld(NWU));

        GetChronoItem_ptr()->Initialize(GetChronoBody2(), GetChronoBody1(), false, chPos2, chPos1, chDir2, chDir1);

    }

    std::shared_ptr<FrConstraintPerpendicular>
    make_constraint_perpendicular(
            const std::shared_ptr<FrAxis>& axis1,
            const std::shared_ptr<FrAxis>& axis2,
            FrOffshoreSystem* system) {

        auto constraint = std::make_shared<FrConstraintPerpendicular>(axis1, axis2, system);
        system->AddLink(constraint);

        return constraint;
    }

    //------------------------------------------------------------------------------------------------------------------

    FrConstraintPlaneOnPlane::FrConstraintPlaneOnPlane(const std::shared_ptr<FrPlane> &plane1,
                                                       const std::shared_ptr<FrPlane> &plane2,
                                                       FrOffshoreSystem *system,
                                                       bool flipped,
                                                       double distance) :
            FrConstraint(plane1->GetNode(), plane2->GetNode(), system), m_plane1(plane1), m_plane2(plane2) {
        m_chronoConstraint = std::make_shared<chrono::ChLinkMatePlane>();
        SetFlipped(flipped);
        SetDistance(distance);
    }

    void FrConstraintPlaneOnPlane::Initialize() {

        auto chPos1 = internal::Vector3dToChVector(m_plane1->GetOriginInWorld(NWU));
        auto chDir1 = internal::Vector3dToChVector(m_plane1->GetNormaleInWorld(NWU));
        auto chPos2 = internal::Vector3dToChVector(m_plane2->GetOriginInWorld(NWU));
        auto chDir2 = internal::Vector3dToChVector(m_plane2->GetNormaleInWorld(NWU));

        GetChronoItem_ptr()->Initialize(GetChronoBody2(), GetChronoBody1(), false, chPos2, chPos1, chDir2, chDir1);

    }

    void FrConstraintPlaneOnPlane::SetFlipped(bool flip) {
        GetChronoItem_ptr()->SetFlipped(flip);
    }

    void FrConstraintPlaneOnPlane::SetDistance(double distance) {
        GetChronoItem_ptr()->SetSeparation(distance);
    }

    std::shared_ptr<FrConstraintPlaneOnPlane>
    make_constraint_plane_on_plane(
            const std::shared_ptr<FrPlane>& plane1,
            const std::shared_ptr<FrPlane>& plane2,
            FrOffshoreSystem* system,
            bool flipped,
            double distance) {

        auto constraint = std::make_shared<FrConstraintPlaneOnPlane>(plane1, plane2, system, flipped, distance);
        system->AddLink(constraint);

        return constraint;
    }

    //------------------------------------------------------------------------------------------------------------------

    FrConstraintPointOnPlane::FrConstraintPointOnPlane(const std::shared_ptr<FrPlane>& plane,
                                                       const std::shared_ptr<FrPoint> &point,
                                                       FrOffshoreSystem *system,
                                                       double distance):
        FrConstraint(plane->GetNode(), point->GetNode(), system), m_plane(plane), m_point(point){
        m_chronoConstraint = std::make_shared<chrono::ChLinkMateXdistance>();
        SetDistance(distance);
    }

    void FrConstraintPointOnPlane::Initialize() {

        auto chPos2 = internal::Vector3dToChVector(m_point->GetPositionInWorld(NWU));
        auto chPos1 = internal::Vector3dToChVector(m_plane->GetOriginInWorld(NWU));
        auto chDir1 = internal::Vector3dToChVector(m_plane->GetNormaleInWorld(NWU));

        GetChronoItem_ptr()->Initialize(GetChronoBody2(), GetChronoBody1(), false, chPos2, chPos1, chDir1);
    }

    void FrConstraintPointOnPlane::SetDistance(double distance) {
        GetChronoItem_ptr()->SetDistance(distance);
    }

    std::shared_ptr<FrConstraintPointOnPlane>
    make_constraint_point_on_plane(
            const std::shared_ptr<FrPlane>& plane,
            const std::shared_ptr<FrPoint>& point,
            FrOffshoreSystem* system,
            double distance) {

        auto constraint = std::make_shared<FrConstraintPointOnPlane>(plane, point, system, distance);
        system->AddLink(constraint);

        return constraint;

    }

    //------------------------------------------------------------------------------------------------------------------

    FrConstraintDistanceToAxis::FrConstraintDistanceToAxis(const std::shared_ptr<FrAxis> &axis,
                                                           const std::shared_ptr<FrPoint> &point,
                                                           FrOffshoreSystem *system,
                                                           bool autoDistance,
                                                           double distance) :
         FrConstraint(axis->GetNode(), point->GetNode(), system), m_point(point), m_axis(axis), m_autoDistance(autoDistance){
        m_chronoConstraint = std::make_shared<chrono::ChLinkRevoluteSpherical>();
        SetDistance(distance);
    }

    void FrConstraintDistanceToAxis::Initialize() {

        auto chPos2 = internal::Vector3dToChVector(m_point->GetPositionInWorld(NWU));
        auto chPos1 = internal::Vector3dToChVector(m_axis->GetOriginInWorld(NWU));
        auto chDir1 = internal::Vector3dToChVector(m_axis->GetDirectionInWorld(NWU));

        GetChronoItem_ptr()->Initialize(GetChronoBody1(), GetChronoBody2(), false, chPos1, chDir1, chPos2, m_autoDistance, GetDistance());

    }

    void FrConstraintDistanceToAxis::SetDistance(double distance) {
        m_distance = distance;
    }

    double FrConstraintDistanceToAxis::GetDistance() const {
        return m_distance;
    }

    std::shared_ptr<FrConstraintDistanceToAxis>
    make_constraint_distance_to_axis(
            const std::shared_ptr<FrAxis>& axis,
            const std::shared_ptr<FrPoint>& point,
            FrOffshoreSystem* system,
            bool autoDistance,
            double distance) {

        auto constraint = std::make_shared<FrConstraintDistanceToAxis>(axis, point, system, autoDistance, distance);
        system->AddLink(constraint);

        return constraint;

    }

    //------------------------------------------------------------------------------------------------------------------

    FrConstraintDistanceBetweenPoints::FrConstraintDistanceBetweenPoints(
            const std::shared_ptr<FrPoint> &point1,
            const std::shared_ptr<FrPoint> &point2,
            FrOffshoreSystem *system,
            bool autoDistance,
            double distance) :
            FrConstraint(point1->GetNode(), point2->GetNode(), system),
            m_point1(point1), m_point2(point2), m_autoDistance(autoDistance){
        m_chronoConstraint = std::make_shared<chrono::ChLinkDistance>();
        SetDistance(distance);
    }


    void FrConstraintDistanceBetweenPoints::SetDistance(double distance) {
        m_distance = distance;
    }

    double FrConstraintDistanceBetweenPoints::GetDistance() const {
        return m_distance;
    }

    void FrConstraintDistanceBetweenPoints::Initialize() {

        auto chPos1 = internal::Vector3dToChVector(m_point1->GetPositionInWorld(NWU));
        auto chPos2 = internal::Vector3dToChVector(m_point2->GetPositionInWorld(NWU));

        GetChronoItem_ptr()->Initialize(GetChronoBody1(), GetChronoBody2(), false, chPos1, chPos2, m_autoDistance, GetDistance());
    }

    std::shared_ptr<FrConstraintDistanceBetweenPoints>
    make_constraint_distance_between_points(const std::shared_ptr<FrPoint>& point1,
                                            const std::shared_ptr<FrPoint>& point2,
                                            FrOffshoreSystem* system,
                                            bool autoDistance,
                                            double distance) {

        auto constraint = std::make_shared<FrConstraintDistanceBetweenPoints>(point1, point2, system, autoDistance, distance);
        system->AddLink(constraint);

        return constraint;

    }
}