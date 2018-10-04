//
// Created by frongere on 11/09/17.
//

#include <frydom/core/FrHydroBody.h>
#include "FrLinearDamping.h"
#include "chrono/physics/ChBody.h"

namespace frydom {


    void FrLinearDamping::UpdateState() {
        /// Body linear velocity expressed in local (body) frame, relatively or not to the current velocity.
        chrono::ChVector<double> linear_vel;
        if (m_relative2Current)
            linear_vel = dynamic_cast<FrHydroBody*>(Body)->GetCurrentRelativeVelocity(NWU,LOCAL);
        else
            linear_vel = Body->TransformDirectionParentToLocal(Body->GetPos_dt());
        /// Body angular velocity expressed in local frame.
        auto angularVelocity = Body->GetWvel_loc();
        /// Convert to eigen vector
        Eigen::VectorXd Velocity = Eigen::VectorXd::Zero(6);
        Velocity(0) = linear_vel.x();
        Velocity(1) = linear_vel.y();
        Velocity(2) = linear_vel.z();
        Velocity(3) = angularVelocity.x();
        Velocity(4) = angularVelocity.y();
        Velocity(5) = angularVelocity.z();
        /// Compute the resulting damping force
        Eigen::VectorXd ResultingForce = Eigen::VectorXd::Zero(6);
        ResultingForce = - m_dampings*Velocity;
        /// Convert back to ChVector
        force.x() = ResultingForce(0);
        force.y() = ResultingForce(1);
        force.z() = ResultingForce(2);
        moment.x() = ResultingForce(3);
        moment.y() = ResultingForce(4);
        moment.z() = ResultingForce(5);
        /// Transforms the linear force back to global frame, but keep angular force (moment) in local frame (imposed by ChForce)
        force = Body->TransformDirectionLocalToParent(force);
    }

    void FrLinearDamping::SetRelative2Current(bool relativeVelocity) {
        m_relative2Current = relativeVelocity;}

    void FrLinearDamping::Initialize() {
        FrForce::Initialize();
        /// YOU CAN'T set m_relative2Current to true, if your body is not at least a FrHydroBody !
        assert(!(m_relative2Current && (dynamic_cast<FrHydroBody*>(Body)== nullptr)));
    }


}  // end namespace frydom