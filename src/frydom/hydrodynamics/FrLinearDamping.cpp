//
// Created by frongere on 11/09/17.
//

#include "MathUtils/Vector6d.h"

#include <frydom/core/FrHydroBody.h> // TODO : Doit dirsparaitre
#include "FrLinearDamping.h"
#include "chrono/physics/ChBody.h" // TODO : Doit disparaitre
#include "frydom/core/FrException.h"

#include "frydom/environment/FrEnvironmentInc.h"


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












    //// REFACTORING ---------->>>>>>>>>>

    FrLinearDamping_::FrLinearDamping_(FLUID_TYPE ft, bool relativeToFluid) : m_fluidType(ft), m_relativeToFluid(relativeToFluid) {
        SetNull();
    }

    void FrLinearDamping_::SetNull() {
        m_dampingMatrix.SetNull();
    }

    void FrLinearDamping_::SetDampingMatrix(const FrLinearDamping_::DampingMatrix &dampingMatrix) {
        m_dampingMatrix = dampingMatrix;
    }

    void FrLinearDamping_::SetDiagonalDamping(double Du, double Dv, double Dw, double Dp, double Dq, double Dr) {
        SetDiagonalTranslationDamping(Du, Dv, Dw);
        SetDiagonalRotationDamping(Dp, Dq, Dr);
    }

    void FrLinearDamping_::SetDiagonalTranslationDamping(double Du, double Dv, double Dw) {
        m_dampingMatrix(0,0) = Du;
        m_dampingMatrix(1,1) = Dv;
        m_dampingMatrix(2,2) = Dw;
    }

    void FrLinearDamping_::SetDiagonalRotationDamping(double Dp, double Dq, double Dr) {
        m_dampingMatrix(3,3) = Dp;
        m_dampingMatrix(4,4) = Dq;
        m_dampingMatrix(5,5) = Dr;
    }

    void FrLinearDamping_::SetDampingCoeff(unsigned int iRow, unsigned int iCol, double coeff) {
        m_dampingMatrix(iRow, iCol) = coeff;
    }

    void FrLinearDamping_::SetRelativeToFluid(bool isRelative) {
        m_relativeToFluid = isRelative;
    }

    bool FrLinearDamping_::GetRelativeToFluid() {return m_relativeToFluid;}

    void FrLinearDamping_::Update(double time) {

        // Body Velocity at COG in body coordinates
        Velocity cogRelVel;
        if (m_relativeToFluid) {  // FIXME : FAUX ici, il faut prendre l'oppose sur l'un ou l'autre
            cogRelVel = m_body->GetLocalRelVelocityInStreamAtCOG(m_fluidType, NWU);
        } else {
            cogRelVel = m_body->GetCOGLocalVelocity(NWU);
        }

        AngularVelocity rotVel = m_body->GetLocalRotationalVelocity(NWU);

        GeneralizedVelocity genRelVel(cogRelVel, rotVel);

        GeneralizedForce genForce = - m_dampingMatrix * genRelVel;

        SetLocalForceTorqueAtCOG(genForce.GetForce(), genForce.GetTorque(), NWU);

    }

    void FrLinearDamping_::Initialize() {
        Check();
    }

    void FrLinearDamping_::StepFinalize() {

    }

    void FrLinearDamping_::Check() const {
        // Here we check if every damping coefficient is positive
        for (unsigned int iRow=0; iRow<6; iRow++) {
            for (unsigned int iCol=0; iCol<6; iCol++) {
                if (m_dampingMatrix(iRow, iCol) < 0.) {
                    throw FrException("Damping coefficients cannot be negative !");
                }
            }
        }
    }

}  // end namespace frydom