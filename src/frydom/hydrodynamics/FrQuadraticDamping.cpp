//
// Created by Lucas Letournel on 12/09/18.
//

#include "frydom/core/FrBody.h"

#include <frydom/core/FrHydroBody.h>
#include "FrQuadraticDamping.h"

#include "chrono/physics/ChBody.h"

#include <cmath>

#include "frydom/environment/FrEnvironment.h"

namespace frydom {


    void FrQuadraticDamping::UpdateState() {

        /// Body linear velocity expressed in local (body) frame, relatively or not to the current velocity.
        chrono::ChVector<double> linear_vel;
        if (m_relative2Current)
            linear_vel = dynamic_cast<FrHydroBody*>(Body)->GetCurrentRelativeVelocity(NWU,LOCAL);
        else
            linear_vel = Body->TransformDirectionParentToLocal(Body->GetPos_dt());

        /// for each component :
        force.x() = m_Su * m_Cu * linear_vel.x() * abs(linear_vel.x());
        force.y() = m_Sv * m_Cv * linear_vel.y() * abs(linear_vel.y());
        force.z() = m_Sw * m_Cw * linear_vel.z() * abs(linear_vel.z());

        /// All components are multiplied by constant : 1/2*rho
        auto rho = dynamic_cast<FrOffshoreSystem*>(Body->GetSystem())->GetEnvironment()->GetWaterDensity();
        force = -0.5*rho*force;

        /// Resulting force is transformed back to global frame
        force = Body->TransformDirectionLocalToParent(force);
    }

    void FrQuadraticDamping::Initialize() {

        assert(!(m_relative2Current && (dynamic_cast<FrHydroBody*>(Body)== nullptr)));
        /// Check that if a damping coefficient is not null, the related section is also not null.
        assert(!(m_Su==0 && m_Cu!=0));
        assert(!(m_Sv==0 && m_Cv!=0));
        assert(!(m_Sw==0 && m_Cw!=0));
        FrForce::Initialize();

    }

    void FrQuadraticDamping::SetRelative2Current(bool relativeVelocity) {
        m_relative2Current = relativeVelocity;
    }









    /// REFACTORING ---------------6>>>>>>>>>>>>>>>>>>>




    FrQuadraticDamping_::FrQuadraticDamping_(std::shared_ptr<FrNode_> node) : FrForce_(node) {}

    void FrQuadraticDamping_::SetDampingCoefficients(double Cu, double Cv, double Cw) {
        m_Cu = Cu;
        m_Cv = Cv;
        m_Cw = Cw;
    }

    void FrQuadraticDamping_::GetDampingCoefficients(double &Cu, double &Cv, double &Cw) {
        Cu = m_Cu;
        Cv = m_Cv;
        Cw = m_Cw;
    }

    void FrQuadraticDamping_::SetProjectedSections(double Su, double Sv, double Sw) {
        m_Su = Su;
        m_Sv = Sv;
        m_Sw = Sw;
    }

    void FrQuadraticDamping_::GetProjectedSections(double &Su, double &Sv, double &Sw) {
        Su = m_Su;
        Sv = m_Sv;
        Sw = m_Sw;
    }

    void FrQuadraticDamping_::SetRelative2Current(bool relativeVelocity) { m_relative2Current = relativeVelocity; }

    bool FrQuadraticDamping_::GetRelative2Current() {return m_relative2Current;}

    void FrQuadraticDamping_::Initialize() {}

    void FrQuadraticDamping_::Update(double time) {

        // Get the relative body velocity with respect to fluid
        auto body = m_node->GetBody();

        double u, v, w;
        body->GetCOGLocalVelocity(u, v, w, NWU);

        // TODO : integrer la vitesse du courant !!

        double rho = 1000;

        SetLocalForce(Force(
                - 0.5 * rho * m_Su * m_Cu * u*fabs(u),
                - 0.5 * rho * m_Sv * m_Cv * v*fabs(v),
                - 0.5 * rho * m_Sw * m_Cw * w*fabs(w)
                ), NWU);

    }

    void FrQuadraticDamping_::StepFinalize() {

    }



}  // end namespace frydom