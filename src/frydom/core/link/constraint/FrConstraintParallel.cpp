//
// Created by lletourn on 22/05/19.
//

#include "FrConstraintParallel.h"

#include "frydom/core/common/FrGeometrical.h"

namespace frydom {


    FrConstraintParallel::FrConstraintParallel(FrAxis *axis1, FrAxis *axis2, FrOffshoreSystem* system) :
    FrLinkBase(axis1->GetNode(), axis2->GetNode(), system), m_axis1(axis1), m_axis2(axis2) {
        m_chronoConstraint = std::make_shared<chrono::ChLinkMateParallel>();
    }

    void FrConstraintParallel::Initialize() {

        auto chPos1 = internal::Vector3dToChVector(m_axis1->GetOriginInWorld(NWU));
        auto chPos2 = internal::Vector3dToChVector(m_axis2->GetOriginInWorld(NWU));
        auto chDir1 = internal::Vector3dToChVector(m_axis1->GetDirectionInWorld(NWU));
        auto chDir2 = internal::Vector3dToChVector(m_axis2->GetDirectionInWorld(NWU));

        m_chronoConstraint->Initialize(GetChronoBody1(), GetChronoBody2(), true, chPos1, chPos2, chDir1, chDir2);

    }

    Force FrConstraintParallel::GetForceInWorld(FRAME_CONVENTION fc) const {
        auto force = internal::ChVectorToVector3d<Force>(m_chronoConstraint->Get_react_force());
        if (IsNED(fc)) internal::SwapFrameConvention(force);
        return force;
    }


} // end namespace frydom