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
        m_volume = MU_PI_4 * diameter * diameter;
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
        m_volume = MU_PI_4 * diameter * diameter;
    }

    void FrSingleElement::SetNodes(FrNode& nodeA, FrNode& nodeB) {
        m_nodeA = std::shared_ptr<FrNode>(&nodeA);
        m_nodeB = std::shared_ptr<FrNode>(&nodeB);
        UpdateFrame();
    }

    void FrSingleElement::SetNodes(std::shared_ptr<FrNode>& nodeA, std::shared_ptr<FrNode>& nodeB) {
        m_nodeA = nodeA;
        m_nodeB = nodeB;
        UpdateFrame();
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
        quat.Q_from_Rotv( chrono::VECT_Z.Cross(direction) );

        m_frame = chrono::ChFrame<>(position, quat);
        m_length = (posB-posA).Length();
        m_dir = direction;

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

        force = rho * (m_ca + 1.) * m_volume * (flow_acceleration - body_acceleration);
        force += 0.5 * m_cd * rho * m_length * velocity * ChAbs(velocity);
        force -= force.Dot(m_dir) * m_dir;          // Delete the axial component
        m_force->SetBodyForce(force);               // Pass force to the FrForce model

        if( relpos.Length() > 0.001 ) {
            moment = relpos.Cross(mybody->TransformPointParentToLocal(force));
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