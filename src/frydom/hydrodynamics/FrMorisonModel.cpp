//
// Created by camille on 17/04/18.
//

#include "chrono/core/ChQuaternion.h"
#include "FrMorisonModel.h"
#include "frydom/core/FrHydroBody.h"
#include "frydom/hydrodynamics/FrMorisonForce.h"
#include "frydom/environment/waves/FrFlowSensor.h"

namespace frydom {

// --------------------------------------------------------------------------
// MORISON ELEMENT
// --------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// SINGLE ELEMENT
// ---------------------------------------------------------------------------

    FrSingleElement::FrSingleElement() {
        m_force = std::make_shared<FrMorisonForce>(this);
    }

    FrSingleElement::FrSingleElement(std::shared_ptr<FrNode>& nodeA,
                                     std::shared_ptr<FrNode>& nodeB,
                                     double diameter,
                                     double ca,
                                     double cd,
                                     double cf)
            : m_ca(ca), m_cd(cd), m_cf(cf), m_diameter(diameter)
    {
        SetNodes(nodeA, nodeB);
        m_force = std::make_shared<FrMorisonForce>(this);
        m_volume = MU_PI_4 * diameter * diameter * m_length;
    }

    FrSingleElement::FrSingleElement(chrono::ChVector<>& posA,
                                     chrono::ChVector<>& posB,
                                     double diameter,
                                     double ca,
                                     double cd,
                                     double cf)
            : m_ca(ca), m_cd(cd), m_cf(cf), m_diameter(diameter)
    {
        m_nodeA = std::make_shared<FrNode>();
        m_nodeB = std::make_shared<FrNode>();

        chrono::ChCoordsys<double> coordA, coordB;
        coordA.pos = posA;
        coordB.pos = posB;
        m_nodeA->Impose_Rel_Coord(coordA);
        m_nodeB->Impose_Rel_Coord(coordB);

        m_nodeA->UpdateState();
        m_nodeB->UpdateState();

        UpdateFrame();

        m_force= std::make_shared<FrMorisonForce>(this);
        m_volume = MU_PI_4 * diameter * diameter * m_length;
    }

    void FrSingleElement::SetNodes(FrNode& nodeA, FrNode& nodeB) {
        m_nodeA = std::shared_ptr<FrNode>(&nodeA);
        m_nodeB = std::shared_ptr<FrNode>(&nodeB);
        UpdateFrame();
        m_volume = MU_PI_4 * m_diameter * m_diameter * m_length;
    }

    void FrSingleElement::SetNodes(std::shared_ptr<FrNode>& nodeA, std::shared_ptr<FrNode>& nodeB) {
        m_nodeA = nodeA;
        m_nodeB = nodeB;
        UpdateFrame();
        m_volume = MU_PI_4 * m_diameter * m_diameter * m_length;
    }

    void FrSingleElement::SetFlowSensor(FrHydroBody* mybody) {
        auto offshore_system = mybody->GetSystem();
        auto waveField = offshore_system->GetEnvironment()->GetFreeSurface()->GetWaveField();
        if (waveField->GetWaveModel()==WAVE_MODEL::LINEAR_WAVES && is_rigid) {
            auto flow_ptr = new FrLinearFlowSensor(waveField.get(), m_frame.GetPos());
            m_flow = std::make_unique<FrLinearFlowSensor>(waveField.get(), m_frame.GetPos());
        } else {
            auto flow_ptr = new FrFlowSensor(waveField.get(), m_frame.GetPos());
            m_flow = std::make_unique<FrFlowSensor>(waveField.get(), m_frame.GetPos());
        }
    }

    void FrSingleElement::UpdateFrame() {

        auto posA = m_nodeA->GetAbsPos();
        auto posB = m_nodeB->GetAbsPos();

        auto position = 0.5*(posA + posB);
        auto direction = posB - posA;
        direction.Normalize();

        auto quat = chrono::ChQuaternion<>();
        auto rotation = chrono::VECT_Z.Cross(direction);
        if (rotation.Length() > 0.) {
            auto angle = asin(rotation.Length());
            rotation.Normalize();
            quat.Q_from_AngAxis(angle, rotation);
        }

        m_frame = chrono::ChFrame<>(position, quat);
        m_length = (posB-posA).Length();
        m_dir = direction;
        m_volume = MU_PI_4 * m_diameter * m_diameter * m_length;

        // CC## Debug
        std::cout << " ----------- Quaternion - ChFrame -------------------- " << std::endl;

        auto CH_VZ = chrono::VECT_Z;
        std::cout << " Vz : " << CH_VZ.x() << ";" << CH_VZ.y() << ";" << CH_VZ.z() << std::endl;

        std::cout << " Quat 1 : " << quat.e0() << ";" << quat.e1() << ";" << quat.e2() << ";" << quat.e3() << std::endl;
        //std::cout << " Quat 2 : " << quat2.e0() << ";" << quat2.e1() << ";" << quat2.e2() << ";" << quat2.e3() << std::endl;

        std::cout << " Node 1 : " << posA.x() << ";" << posA.y() << ";" << posA.z() << std::endl;
        std::cout << " Node 2 : " << posB.x() << ";" << posB.y() << ";" << posB.z() << std::endl;
        std::cout << " Position : " << position.x() << ";" << position.y() << ";" << position.z() << std::endl;
        std::cout << " Direction : " << direction.x() << ";" << direction.y() << ";" << direction.z() << std::endl;
        std::cout << " Rotation : " << rotation.x() << ";" << rotation.y() << ";" << rotation.z() << std::endl;
        //std::cout << " angle = " << angle << std::endl;
        std::cout << " CH_C_PI_2 : " << chrono::CH_C_PI_2 << std::endl;

        auto VX = m_frame.GetA().Get_A_Xaxis();
        auto VY = m_frame.GetA().Get_A_Yaxis();
        auto VZ = m_frame.GetA().Get_A_Zaxis();

        std::cout << " frame x-axis : " << VX.x() << ";" << VX.y() << ";" << VX.z() << std::endl;
        std::cout << " frame y-axis : " << VY.x() << ";" << VY.y() << ";" << VY.z() << std::endl;
        std::cout << " frame z-axis : " << VZ.x() << ";" << VZ.y() << ";" << VZ.z() << std::endl;

        // ##CC


    }

    void FrSingleElement::AddForce(FrHydroBody* body) {
        body->AddForce(m_force);
    }

    chrono::ChVector<double> FrSingleElement::GetBodyForce() const {
        return m_force->GetBodyForce();
    }

    chrono::ChVector<double> FrSingleElement::GetBodyTorque() const {
        return m_force->GetBodyTorque();
    }

    void FrSingleElement::SetBody(FrHydroBody* body, bool add_force) {

        body->AddNode(m_nodeA);
        body->AddNode(m_nodeB);

        m_nodeA->UpdateState();
        m_nodeB->UpdateState();

        // Update element position
        UpdateFrame();

        m_force->SetBody(body);

        if (add_force) { body->AddForce(m_force); }
    }

    inline double FrSingleElement::WaterDensity() const {
        auto mybody = m_force->GetBody();
        auto offshore_system = dynamic_cast<FrOffshoreSystem *>(mybody->GetSystem());
        return offshore_system->GetEnvironment()->GetWaterDensity();
    }

    void FrSingleElement::UpdateState() {

        chrono::ChVector<double> force;
        chrono::ChVector<double> moment;

        auto mybody = dynamic_cast<FrHydroBody*>(m_force->GetBody());

        m_nodeA->UpdateState();
        m_nodeB->UpdateState();

        if (!is_rigid) {
            UpdateFrame();
            m_flow->SetPos(m_frame.GetPos());
        }


        auto relpos = mybody->TransformPointParentToLocal(m_frame.GetPos());
        auto body_acceleration = mybody->PointAccelerationLocalToParent(relpos);

        auto rho = WaterDensity();
        auto flow_velocity = m_flow->GetVelocity();
        auto flow_acceleration = m_flow->GetAcceleration();
        auto current_relative_velocity = mybody->GetCurrentRelativeVelocity(relpos);
        auto velocity = flow_velocity + current_relative_velocity;

        // ##CC
        std::cout << " ------------- Morison velocity -------------------- " << std::endl;

        std::cout << " velocity : " << velocity.x() << ";" << velocity.y() << ";" << velocity.z() << std::endl;
        std::cout << " body acceleration : " << body_acceleration.x() << ";"
                  << body_acceleration.y() << ";" << body_acceleration.z() << std::endl;
        std::cout << " flow acceleration : " << flow_acceleration.x() << ";"
                  << flow_acceleration.y() << ";" << flow_acceleration.z() << std::endl;

        // ##CC

        velocity = m_frame.TransformDirectionParentToLocal(velocity);
        flow_acceleration = m_frame.TransformDirectionParentToLocal(flow_acceleration);
        body_acceleration = m_frame.TransformDirectionParentToLocal(body_acceleration);

        // ##CC

        std::cout << " => Change reference frame : Parent to local" << std::endl;

        std::cout << " velocity : " << velocity.x() << ";" << velocity.y() << ";" << velocity.z() << std::endl;
        std::cout << " body acceleration : " << body_acceleration.x() << ";"
                  << body_acceleration.y() << ";" << body_acceleration.z() << std::endl;
        std::cout << " flow acceleration : " << flow_acceleration.x() << ";"
                  << flow_acceleration.y() << ";" << flow_acceleration.z() << std::endl;

        // ##CC

        force = rho * (m_ca + 1.) * m_volume * (flow_acceleration - body_acceleration);
        force.z() = 0.;
        force.x() += 0.5 * m_cd * rho * m_length * velocity.x() * std::abs(velocity.x());
        force.y() += 0.5 * m_cd * rho * m_length * velocity.y() * std::abs(velocity.y());


        // ##CC
        std::cout << " ---------------- Morison force ---------------------- " << std::endl;
        std::cout << " rho = " << rho << std::endl;
        std::cout << " ca = " << m_ca << "; cd = " << m_cd << "; length = " << m_length
                  << " volume = " << m_volume << std::endl;
        std::cout << " force : " << force.x() << ";" << force.y() << ";" << force.z() << std::endl;
        // ##CC

        //force -= force.Dot(m_dir) * m_dir;          // Delete the axial component
        force = m_frame.TransformDirectionLocalToParent(force);

        // ##CC
        std::cout << " => change reference frame : Local to parent" << std::endl;
        std::cout << " force : " << force.x() << ";" << force.y() << ";" << force.z() << std::endl;
        // ##CC

        m_force->SetBodyForce(force);               // Pass force to the FrForce model

        if( relpos.Length() > 0.001 ) {
            moment = relpos.Cross(mybody->TransformDirectionParentToLocal(force));
        } else {
            moment = chrono::ChVector<double>(0.);
        }
        m_force->SetBodyTorque(moment);

    }

    void FrSingleElement::Initialize() {
        auto body = dynamic_cast<FrHydroBody*>(m_force->GetBody());
        SetFlowSensor(body);
    }

// -----------------------------------------------------------------------------
// COMPOSITE ELEMENT
// -----------------------------------------------------------------------------

    FrCompositeElement::FrCompositeElement() {
        m_force = std::make_shared<FrMorisonForce>(this);
    }

    void FrCompositeElement::AddElement(chrono::ChVector<> posA,
                                        chrono::ChVector<> posB,
                                        double diameter,
                                        double ca, double cd, double cf) {
        m_morison.push_back(std::make_unique<FrSingleElement>(posA, posB, diameter, ca, cd, cf));
    }

    void FrCompositeElement::AddElement(chrono::ChVector<> posA,
                                        chrono::ChVector<> posB) {
        m_morison.push_back(std::make_unique<FrSingleElement>(posA, posB, m_diameter, m_ca, m_cd, m_cf));
    }

    void FrCompositeElement::AddElement(std::shared_ptr<FrNode>& nodeA,
                                        std::shared_ptr<FrNode>& nodeB,
                                        double diameter, double ca, double cd, double cf) {
        m_morison.push_back(std::make_unique<FrSingleElement>(nodeA, nodeB, diameter, ca, cd, cf));
    }

    void FrCompositeElement::AddElement(std::shared_ptr<FrNode>& nodeA,
                                        std::shared_ptr<FrNode>& nodeB) {
        m_morison.push_back(std::make_unique<FrSingleElement>(nodeA, nodeB, m_diameter, m_ca, m_cd, m_cf));
    }

    void FrCompositeElement::AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const double dL,
                                        double diameter, double ca, double cd, double cf) {

        auto vect = posB - posA;
        auto length = vect.Length();
        auto dir = vect/length;
        auto n = int(length/dL);
        auto dl = length/double(n);

        chrono::ChVector<double> pos1;
        chrono::ChVector<double> pos2;
        pos2 = posA;
        for (unsigned int i=0; i<n; ++i) {
            pos1 = pos2;
            pos2 = pos1 + dl*dir;
            AddElement(pos1, pos2, diameter, ca, cd ,cf);
        }

    }

    void FrCompositeElement::AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const double dL) {
        AddElement(posA, posB, dL, m_diameter, m_ca, m_cd, m_cf);
    }

    void FrCompositeElement::AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const int n,
                                        double diameter, double ca, double cd, double cf) {

        auto vect = posB - posA;
        auto length = vect.Length();
        auto dir = vect/length;
        auto dl = length/double(n);

        chrono::ChVector<double> pos1;
        chrono::ChVector<double> pos2;
        pos2 = posA;
        for (unsigned int i=0; i<n; ++i) {
            pos1 = pos2;
            pos2 = pos1 + dl*dir;
            AddElement(pos1, pos2, diameter, ca, cd, cf);
        }

    }

    void FrCompositeElement::AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const int n) {
        AddElement(posA, posB, n, m_diameter, m_ca, m_cd, m_cf);
    }

    void FrCompositeElement::Initialize() {
        for (auto& element : m_morison) {
            element->Initialize();
        }
    }

    void FrCompositeElement::AddForce(FrHydroBody* body) {
        if (!is_global_force) {
            for (auto& element : m_morison) {
                element->AddForce(body);
            }
        } else {
            body->AddForce(m_force);
        }
    }

    void FrCompositeElement::UpdateState() {
        for (auto& element: m_morison) {
            element->UpdateState();
        }
        if(is_global_force) {
            m_force->SetBodyForce( GetBodyForce() );
            m_force->SetBodyTorque( GetBodyTorque() );
        }
    }
}