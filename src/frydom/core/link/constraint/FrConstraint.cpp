//
// Created by lletourn on 22/05/19.
//

#include "FrConstraint.h"

#include "frydom/core/common/FrGeometrical.h"
#include "frydom/core/common/FrNode.h"

namespace frydom {


    FrConstraint::FrConstraint(const std::shared_ptr<FrNode> &node1, const std::shared_ptr<FrNode> &node2,
                               FrOffshoreSystem *system) : FrLinkBase(node1, node2, system) {

    }

    Force FrConstraint::GetForceInNode2(FRAME_CONVENTION fc) const {
        auto force = internal::ChVectorToVector3d<Force>(m_chronoConstraint->Get_react_force());
        if (IsNED(fc)) internal::SwapFrameConvention(force);
        return force;
    }

    Torque FrConstraint::GetTorqueInNode2(FRAME_CONVENTION fc) const {
        auto torque = internal::ChVectorToVector3d<Torque>(m_chronoConstraint->Get_react_torque());
        if (IsNED(fc)) internal::SwapFrameConvention(torque);
        return torque;
    }

    Force FrConstraint::GetForceInWorld(FRAME_CONVENTION fc) const {
        return m_node2->ProjectVectorInWorld(GetForceInNode2(fc), fc);
    }

    Torque FrConstraint::GetTorqueInWorldAtLink(FRAME_CONVENTION fc) const {
        return m_node2->ProjectVectorInWorld(GetTorqueInNode2(fc), fc);
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

    chrono::ChLinkMateGeneric *FrConstraint::GetChronoItem_ptr() const {
        return m_chronoConstraint.get();
    }

    //------------------------------------------------------------------------------------------------------------------

    FrConstraintParallel_::FrConstraintParallel_(const std::shared_ptr<FrAxis>& axis1, const std::shared_ptr<FrAxis>& axis2, FrOffshoreSystem* system) :
            FrConstraint(axis1->GetNode(), axis2->GetNode(), system), m_axis1(axis1), m_axis2( axis2) {
            m_chronoConstraint = std:: make_shared<chrono::ChLinkMateParallel>();
    }

    void FrConstraintParallel_::Initialize() {

        auto chPos1 = internal::Vector3dToChVector(m_axis1->GetOriginInWorld(NWU));
        auto chPos2 = internal::Vector3dToChVector(m_axis2->GetOriginInWorld(NWU));
        auto chDir1 = internal::Vector3dToChVector(m_axis1->GetDirectionInWorld(NWU));
        auto chDir2 = internal::Vector3dToChVector(m_axis2->GetDirectionInWorld(NWU));

        m_chronoConstraint->Initialize(GetChronoBody2(), GetChronoBody1(), false, chPos2, chPos1, chDir2, chDir1);

    }

    //------------------------------------------------------------------------------------------------------------------

    FrConstraintPerpendicular::FrConstraintPerpendicular(const std::shared_ptr<FrAxis>& axis1, const std::shared_ptr<FrAxis>& axis2, FrOffshoreSystem* system) :
            FrConstraint(axis1->GetNode(), axis2->GetNode(), system), m_axis1(axis1), m_axis2( axis2) {
        m_chronoConstraint = std:: make_shared<chrono::ChLinkMateOrthogonal>();
    }

    void FrConstraintPerpendicular::Initialize() {

        auto chPos1 = internal::Vector3dToChVector(m_axis1->GetOriginInWorld(NWU));
        auto chPos2 = internal::Vector3dToChVector(m_axis2->GetOriginInWorld(NWU));
        auto chDir1 = internal::Vector3dToChVector(m_axis1->GetDirectionInWorld(NWU));
        auto chDir2 = internal::Vector3dToChVector(m_axis2->GetDirectionInWorld(NWU));

        m_chronoConstraint->Initialize(GetChronoBody2(), GetChronoBody1(), false, chPos2, chPos1, chDir2, chDir1);

    }

    //------------------------------------------------------------------------------------------------------------------

    FrConstraintPlaneOnPlane::FrConstraintPlaneOnPlane(const std::shared_ptr<FrPlane> &plane1,
                                                       const std::shared_ptr<FrPlane> &plane2,
                                                       FrOffshoreSystem *system) :
            FrConstraint(plane1->GetNode(), plane2->GetNode(), system), m_plane1(plane1), m_plane2(plane2) {
        m_chronoConstraint = std::make_shared<chrono::ChLinkMatePlane>();
    }

    void FrConstraintPlaneOnPlane::Initialize() {

        auto chPos1 = internal::Vector3dToChVector(m_plane1->GetOriginInWorld(NWU));
        auto chPos2 = internal::Vector3dToChVector(m_plane2->GetOriginInWorld(NWU));
        auto chDir1 = internal::Vector3dToChVector(m_plane1->GetNormaleInWorld(NWU));
        auto chDir2 = internal::Vector3dToChVector(m_plane2->GetNormaleInWorld(NWU));

        GetChronoItem_ptr()->Initialize(GetChronoBody2(), GetChronoBody1(), false, chPos2, chPos1, chDir2, chDir1);

//        GetChronoItem_ptr()->SetSeparation(2.);
    }

    void FrConstraintPlaneOnPlane::SetFlipped(bool flip) {
        GetChronoItem_ptr()->SetFlipped(flip);
    }

    void FrConstraintPlaneOnPlane::SetDistance(double distance) {
        GetChronoItem_ptr()->SetSeparation(distance);
    }

    //------------------------------------------------------------------------------------------------------------------

    FrConstraintPointOnPlane::FrConstraintPointOnPlane(const std::shared_ptr<FrPoint> &point,
                                                       const std::shared_ptr<FrPlane>& plane,
                                                           FrOffshoreSystem *system,
                                                           double distance):
        FrConstraint(plane->GetNode(), point->GetNode(), system), m_plane(plane), m_point(point){
        m_chronoConstraint = std::make_shared<chrono::ChLinkMateXdistance>();
        SetDistance(distance);
    }

    void FrConstraintPointOnPlane::Initialize() {

        auto chPos1 = internal::Vector3dToChVector(m_plane->GetOriginInWorld(NWU));
        auto chPos2 = internal::Vector3dToChVector(m_point->GetPositionInWorld(NWU));
        auto chDir1 = internal::Vector3dToChVector(m_plane->GetNormaleInWorld(NWU));

        GetChronoItem_ptr()->Initialize(GetChronoBody2(), GetChronoBody1(), false, chPos2, chPos1, chDir1);

//        auto chDir2 = internal::Vector3dToChVector(m_axis->GetNode()->GetFrameInWorld().GetZAxisInParent(NWU));
//
//        m_chronoConstraint->Initialize(GetChronoBody2(), GetChronoBody1(), false, chPos2, chPos1, chDir2, chDir1);

//        auto pointFrame = internal::FrFrame2ChFrame(m_point->GetNode()->GetFrameInWorld());
//        auto axisFrame = internal::FrFrame2ChFrame(m_axis->GetNode()->GetFrameInWorld());
//
//        m_chronoConstraint->Initialize(GetChronoBody2(),GetChronoBody1(),true,pointFrame,axisFrame);

    }

    void FrConstraintPointOnPlane::SetDistance(double distance) {
        GetChronoItem_ptr()->SetDistance(distance);
    }
}