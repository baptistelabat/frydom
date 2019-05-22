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
            FrConstraint(axis1->GetNode(), axis2->GetNode(), system), m_axis1(axis1), m_axis2( axis2){
            m_chronoConstraint = std:: make_shared<chrono::ChLinkMateParallel>();
    }

    void FrConstraintParallel_::Initialize() {

        auto chPos1 = internal::Vector3dToChVector(m_axis1->GetOriginInWorld(NWU));
        auto chPos2 = internal::Vector3dToChVector(m_axis2->GetOriginInWorld(NWU));
        auto chDir1 = internal::Vector3dToChVector(m_axis1->GetDirectionInWorld(NWU));
        auto chDir2 = internal::Vector3dToChVector(m_axis2->GetDirectionInWorld(NWU));

        m_chronoConstraint->Initialize(GetChronoBody1(), GetChronoBody2(), false, chPos1, chPos2, chDir1, chDir2);

    }

    //------------------------------------------------------------------------------------------------------------------

    FrConstraintPerpendicular::FrConstraintPerpendicular(const std::shared_ptr<FrAxis>& axis1, const std::shared_ptr<FrAxis>& axis2, FrOffshoreSystem* system) :
            FrConstraint(axis1->GetNode(), axis2->GetNode(), system), m_axis1(axis1), m_axis2( axis2){
        m_chronoConstraint = std:: make_shared<chrono::ChLinkMateOrthogonal>();
    }

    void FrConstraintPerpendicular::Initialize() {

        auto chPos1 = internal::Vector3dToChVector(m_axis1->GetOriginInWorld(NWU));
        auto chPos2 = internal::Vector3dToChVector(m_axis2->GetOriginInWorld(NWU));
        auto chDir1 = internal::Vector3dToChVector(m_axis1->GetDirectionInWorld(NWU));
        auto chDir2 = internal::Vector3dToChVector(m_axis2->GetDirectionInWorld(NWU));

        m_chronoConstraint->Initialize(GetChronoBody1(), GetChronoBody2(), true, chPos1, chPos2, chDir1, chDir2);

    }
}