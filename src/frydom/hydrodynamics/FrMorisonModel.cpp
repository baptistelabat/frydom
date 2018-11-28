//
// Created by camille on 17/04/18.
//

#include "chrono/core/ChQuaternion.h"
#include "FrMorisonModel.h"
#include "frydom/core/FrHydroBody.h"
#include "frydom/hydrodynamics/FrMorisonForce.h"
#include "frydom/environment/ocean/freeSurface/waves/FrFlowSensor.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"

#include "frydom/core/FrVector.h"
#include "frydom/environment/ocean/FrOceanInc.h"
#include "frydom/environment/flow/FrFlowInc.h"

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
            : m_ca_x(ca), m_ca_y(ca), m_cd_x(cd), m_cd_y(cd), m_cf(cf), m_diameter(diameter)
    {
        SetNodes(nodeA, nodeB);
        m_force = std::make_shared<FrMorisonForce>(this);
        m_volume = MU_PI_4 * diameter * diameter * m_length;
    }

    FrSingleElement::FrSingleElement(std::shared_ptr<FrNode>& nodeA,
                                     std::shared_ptr<FrNode>& nodeB,
                                     double diameter,
                                     double ca_x, double ca_y,
                                     double cd_x, double cd_y,
                                     double cf)
            : m_ca_x(ca_x), m_ca_y(ca_y), m_cd_x(cd_x), m_cd_y(cd_y), m_cf(cf), m_diameter(diameter)
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
            : m_ca_x(ca), m_ca_y(ca), m_cd_x(cd), m_cd_y(cd), m_cf(cf), m_diameter(diameter)
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

    FrSingleElement::FrSingleElement(chrono::ChVector<>& posA,
                                     chrono::ChVector<>& posB,
                                     double diameter,
                                     double ca_x, double ca_y,
                                     double cd_x, double cd_y,
                                     double cf)
            : m_ca_x(ca_x), m_ca_y(ca_y), m_cd_x(cd_x), m_cd_y(cd_y), m_cf(cf), m_diameter(diameter)
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

        chrono::ChVector<double> pos;
        if (waveField->GetWaveModel()==WAVE_MODEL::LINEAR_WAVES && is_rigid) {
            pos = m_frame.GetPos();
//            auto flow_ptr = new FrLinearFlowSensor(waveField, pos[0], pos[1], pos[2]);
            m_flow = std::make_unique<FrLinearFlowSensor>(waveField, pos[0], pos[1], pos[2]);

        } else {
            // FIXME : ne peut pas instancier FrFlowSensor --> classe passe abstraite
//            auto flow_ptr = new FrFlowSensor(waveField, m_frame.GetPos());
//            m_flow = std::make_unique<FrFlowSensor>(waveField, pos[0], pos[1], pos[2]);
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
        rotation.Normalize();

        /**
        if (rotation.Length() > 0.) {
            auto angle = asin(rotation.Length());
            rotation.Normalize();
            quat.Q_from_AngAxis(angle, rotation);
        }
        m_frame = chrono::ChFrame<>(position, quat);
        **/

        auto e2 = direction.Cross(rotation);

        auto A_matrix = chrono::ChMatrix33<double>();
        A_matrix.Set_A_axis(rotation, e2, direction);
        m_frame = chrono::ChFrame<>(position, A_matrix);

        m_length = (posB-posA).Length();
        m_dir = direction;
        m_volume = MU_PI_4 * m_diameter * m_diameter * m_length;

        //##CC
        /**
        auto xaxis = m_frame.GetA().Get_A_Xaxis();
        auto yaxis = m_frame.GetA().Get_A_Yaxis();
        auto zaxis = m_frame.GetA().Get_A_Zaxis();
        std::cout << "Morison Element : "<< std::endl;
        std::cout << "A : [ " << posA.x() << " ; " << posA.y() << " ; " << posA.z() << " ]" << std::endl;
        std::cout << "B : [ " << posB.x() << " ; " << posB.y() << " ; " << posB.z() << " ]" << std::endl;
        std::cout << "center frame : [ " << position.x() << " ; " << position.y() << " ; " << position.z() << " ]" << std::endl;
        std::cout << "x-axis : [ " << xaxis.x() << " ; " << xaxis.y() << " ; " << xaxis.z() << " ]" << std::endl;
        std::cout << "y-axis : [ " << yaxis.x() << " ; " << yaxis.y() << " ; " << yaxis.z() << " ]" << std::endl;
        std::cout << "z-axis : [ " << zaxis.x() << " ; " << zaxis.y() << " ; " << zaxis.z() << " ]" << std::endl;
        **/
        //##CC


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

        auto pos = m_frame.GetPos();

        if (!is_rigid) {
            UpdateFrame();
            m_flow->SetPos(pos[0], pos[1], pos[2]);
        }


        auto relpos = mybody->TransformPointParentToLocal(pos);
        auto body_acceleration = mybody->PointAccelerationLocalToParent(relpos);

        auto rho = WaterDensity();
        auto flow_velocity = m_flow->GetVelocity();
        auto flow_acceleration = m_flow->GetAcceleration();

        chrono::ChVector<double> current_relative_velocity;
        if (m_include_current) {
            current_relative_velocity = mybody->GetCurrentRelativeVelocity(relpos);
        } else {
            current_relative_velocity = -mybody->PointSpeedLocalToParent(relpos);
        }
        auto velocity = flow_velocity + current_relative_velocity;

        velocity = m_frame.TransformDirectionParentToLocal(velocity);
        flow_acceleration = m_frame.TransformDirectionParentToLocal(flow_acceleration);
        body_acceleration = m_frame.TransformDirectionParentToLocal(body_acceleration);

        force.x() = rho * (m_ca_x + 1.) * m_volume * (flow_acceleration.x() - body_acceleration.x());
        force.y() = rho * (m_ca_y + 1.) * m_volume * (flow_acceleration.y() - body_acceleration.y());
        force.z() = 0.;
        force.x() += 0.5 * m_cd_x * rho * m_diameter * m_length * velocity.x() * std::abs(velocity.x());
        force.y() += 0.5 * m_cd_y * rho * m_diameter * m_length * velocity.y() * std::abs(velocity.y());

        force = m_frame.TransformDirectionLocalToParent(force);
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
                                        chrono::ChVector<> posB,
                                        double diameter,
                                        double ca_x, double ca_y, double cd_x, double cd_y, double cf) {
        m_morison.push_back(std::make_unique<FrSingleElement>(posA, posB, diameter, ca_x, ca_y, cd_x, cd_y, cf));
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
                                        std::shared_ptr<FrNode>& nodeB,
                                        double diameter, double ca_x, double ca_y, double cd_x, double cd_y, double cf) {
        m_morison.push_back(std::make_unique<FrSingleElement>(nodeA, nodeB, diameter, ca_x, ca_y, cd_x, cd_y, cf));
    }

    void FrCompositeElement::AddElement(std::shared_ptr<FrNode>& nodeA,
                                        std::shared_ptr<FrNode>& nodeB) {
        m_morison.push_back(std::make_unique<FrSingleElement>(nodeA, nodeB, m_diameter, m_ca, m_cd, m_cf));
    }

    void FrCompositeElement::AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const double dL,
                                        double diameter, double ca, double cd, double cf) {
        AddElement(posA, posB, dL, diameter, ca, ca, cd, cd, cf);
    }


    void FrCompositeElement::AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const double dL,
                                        double diameter, double ca_x, double ca_y, double cd_x, double cd_y, double cf) {

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
            AddElement(pos1, pos2, diameter, ca_x, ca_y, cd_x, cd_y, cf);
        }
    }

    void FrCompositeElement::AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const double dL) {
        AddElement(posA, posB, dL, m_diameter, m_ca, m_cd, m_cf);
    }

    void FrCompositeElement::AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const int n,
                                        double diameter, double ca, double cd, double cf) {
        AddElement(posA, posB, diameter, n, ca, ca, cd, cd, cf);
    }


    void FrCompositeElement::AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const int n,
                                        double diameter, double ca_x, double ca_y, double cd_x, double cd_y, double cf) {

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
            AddElement(pos1, pos2, diameter, ca_x, ca_y, cd_x, cd_y, cf);
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



















    /// <<<<<<<<<<<<<<<<<<<<<<<<<<<< REFACTORING


    // -----------------------------------------------------------------
    // MORISON MODEL
    // -----------------------------------------------------------------

    void FrMorisonElement_::SetFrame(FrBody_* body, Position posA, Position posB, Direction vect) {

        Direction position = 0.5*(posA + posB);

        Direction e3 = posB - posA;
        e3.normalize();
        Direction e1 = vect.cross(e3);

        if (std::abs(e1.norm()) < FLT_EPSILON) {
            e1.normalize();
        } else {
            e1 = Direction(1., 0., 0.);
        }

        Direction e2 = e3.cross(e1);
        e2.normalize();

        m_frame->Set(body, position, e1, e2, e3);

    }

    void FrMorisonElement_::SetFrame(FrBody_* body, FrFrame_ frame) {
        m_frame = std::make_unique<FrNode_>(body, frame);
    }

    Force FrMorisonElement_::GetForceInWorld(FRAME_CONVENTION fc) const {
        auto force = m_force;
        if (IsNED(fc)) internal::SwapFrameConvention(force);
        return force;
    }

    Torque FrMorisonElement_::GetTorqueInBody() const {
        return m_torque;
    }


    // ---------------------------------------------------------------------
    // MORISON SINGLE ELEMENT
    // ---------------------------------------------------------------------

    FrMorisonSingleElement_::FrMorisonSingleElement_(FrBody_* body) {
        m_frame = std::make_shared<FrNode_>(body);
    }

    FrMorisonSingleElement_::FrMorisonSingleElement_(FrBody_* body, Position posA, Position posB, double diameter,
                                                     MorisonCoeff ca, MorisonCoeff cd, double cf,
                                                     Direction perpendicular) {
        SetAddedMass(ca);
        SetDragCoeff(cd);
        SetFrictionCoeff(cf);

        m_frame = std::make_shared<FrNode_>(body);
        SetFrame(body, posA, posB, perpendicular);

        SetDiameter(diameter);
        SetLength(posA, posB);
        SetVolume();
    }

    FrMorisonSingleElement_::FrMorisonSingleElement_(std::shared_ptr<FrNode_>& nodeA,
                                                     std::shared_ptr<FrNode_>& nodeB,
                                                     double diameter, MorisonCoeff ca, MorisonCoeff cd, double cf,
                                                     Direction perpendicular) {
        SetNodes(nodeA, nodeB);

        SetAddedMass(ca);
        SetDragCoeff(cd);
        SetFrictionCoeff(cf);

        m_frame = std::make_shared<FrNode_>(nodeA->GetBody());
        SetFrame(nodeA->GetBody(), nodeA->GetPositionInWorld(NWU), nodeB->GetPositionInWorld(NWU), perpendicular);

        SetDiameter(diameter);
        SetLength(nodeA->GetPositionInWorld(NWU), nodeB->GetPositionInWorld(NWU));
        SetVolume();
    }

    FrMorisonSingleElement_::FrMorisonSingleElement_(FrBody_* body, FrFrame_ frame, double diameter, double length,
                                                     MorisonCoeff ca, MorisonCoeff cd, double cf) {
        SetAddedMass(ca);
        SetDragCoeff(cd);
        SetFrictionCoeff(cf);

        SetFrame(body, frame);

        SetDiameter(diameter);
        SetLength(length);
        SetVolume();
    }


    void FrMorisonSingleElement_::SetNodes(std::shared_ptr<FrNode_>& nodeA, std::shared_ptr<FrNode_>& nodeB) {
        m_nodeA = nodeA;
        m_nodeB = nodeB;
    }

    void FrMorisonSingleElement_::SetAddedMass(MorisonCoeff ca) {
        assert(ca.x >= -FLT_EPSILON or std::abs(ca.x) <= FLT_EPSILON);
        assert(ca.y >= -FLT_EPSILON or std::abs(ca.y) <= FLT_EPSILON);
        m_property.ca = ca;
    }

    void FrMorisonSingleElement_::SetDragCoeff(MorisonCoeff cd) {
        assert(cd.x >= -FLT_EPSILON or std::abs(cd.x) <= FLT_EPSILON);
        assert(cd.y >= -FLT_EPSILON or std::abs(cd.y) <= FLT_EPSILON);
        m_property.cd = cd;
    }

    void FrMorisonSingleElement_::SetFrictionCoeff(double cf) {
        assert(cf >= -FLT_EPSILON or std::abs(cf) <= FLT_EPSILON);
        m_property.cf = cf;
    }

    void FrMorisonSingleElement_::SetDiameter(const double diameter) {
        assert(diameter >= -FLT_EPSILON or std::abs(diameter) <= FLT_EPSILON);
        m_property.diameter = diameter;
    }



    void FrMorisonSingleElement_::SetLength(Position posA, Position posB) {
        m_property.length = (posB - posA).norm();
    }

    void FrMorisonSingleElement_::SetVolume() {
        m_property.volume = MU_PI_4 * GetDiameter() * GetDiameter() * GetLength();
    }


    Velocity FrMorisonSingleElement_::GetFlowVelocity() {

        Velocity velocity;
        Position worldPos = m_frame->GetPositionInWorld(NWU);
        auto body = m_frame->GetBody();

        auto waveField = GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

        velocity = waveField->GetVelocity(worldPos);
        velocity -= m_frame->GetVelocityInWorld(NWU);

        if (m_includeCurrent) {
            velocity += GetSystem()->GetEnvironment()->GetOcean()->GetCurrent()->GetFluxVelocityInWorld(worldPos, NWU);
        }

        Velocity velocityBody = body->GetFrame().ProjectVectorParentInFrame(velocity);
        return m_frame->GetFrame().ProjectVectorParentInFrame(velocityBody);
    }

    Acceleration FrMorisonSingleElement_::GetFlowAcceleration() {

        Acceleration acceleration;
        Position worldPos = m_frame->GetPositionInWorld(NWU);
        auto body = m_frame->GetBody();

        auto waveField = GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

        acceleration = waveField->GetAcceleration(worldPos);
        acceleration -= m_frame->GetAccelerationInWorld(NWU);

        Acceleration accBody = body->GetFrame().ProjectVectorParentInFrame(acceleration);
        return m_frame->GetFrame().ProjectVectorParentInFrame(accBody);
    }

    //
    // UPDATE
    //

    void FrMorisonSingleElement_::Update(double time) {

        Force localForce;

        auto rho = GetSystem()->GetEnvironment()->GetOcean()->GetDensity();
        auto body = m_frame->GetBody();

        Velocity velocity = GetFlowVelocity();
        localForce.x() = 0.5 * m_property.cd.x * rho * m_property.diameter * m_property.length * velocity.x() * std::abs(velocity.x());
        localForce.y() = 0.5 * m_property.cd.y * rho * m_property.diameter * m_property.length * velocity.y() * std::abs(velocity.y());

        if (m_extendedModel) {
            Acceleration acceleration = GetFlowAcceleration();
            localForce.x() += rho * (m_property.ca.x + 1.) * GetVolume() * acceleration.x();
            localForce.y() += rho * (m_property.ca.y + 1.) * GetVolume() * acceleration.y();
        }

        localForce.z() = 0.; // TODO : friction

        // Project force in world at COG
        auto forceBody = m_frame->GetFrame().ProjectVectorInParent(localForce);
        m_force = body->GetFrame().ProjectVectorInParent(forceBody);

        //Project torque in body at COG
        Position relPos = m_frame->GetNodePositionInBody(NWU) - body->GetCOG(NWU);
        m_torque = relPos.cross(forceBody);
    }

    void FrMorisonSingleElement_::Initialize() {

    }

    void FrMorisonSingleElement_::StepFinalize() {

    }


    // -------------------------------------------------------------------
    // MORISON COMPOSITE FORCE MODEL
    // -------------------------------------------------------------------

    FrMorisonCompositeElement_::FrMorisonCompositeElement_(FrBody_* body) {
        m_frame = std::make_shared<FrNode_>(body);
    }

    FrMorisonCompositeElement_::FrMorisonCompositeElement_(FrBody_* body, FrFrame_& frame) {
        m_frame = std::make_shared<FrNode_>(body, frame);
    }

    void FrMorisonCompositeElement_::AddElement(std::shared_ptr<FrNode_> nodeA, std::shared_ptr<FrNode_> nodeB, double diameter,
                                                MorisonCoeff ca, MorisonCoeff cd, double cf, Direction perpendicular) {
        m_morison.push_back(std::make_unique<FrMorisonSingleElement_>(nodeA, nodeB, diameter, ca, cd, cf, perpendicular));
    }

    void FrMorisonCompositeElement_::AddElement(Position posA, Position posB, double diameter,
                                                MorisonCoeff ca, MorisonCoeff cd, double cf, unsigned int n,
                                                Direction perpendicular) {
        Direction dV = (posB - posA) / n;

        Position pos;
        for (unsigned int i=0; i<n; ++i) {
            pos = posA + dV * n;
            m_morison.push_back(std::make_unique<FrMorisonSingleElement_>(m_frame->GetBody(), pos, pos + dV, diameter,
                                                                          ca, cd, cf, perpendicular));
        }
    }

    void FrMorisonCompositeElement_::AddElement(FrFrame_ frame, double diameter, double length,
                                                MorisonCoeff ca, MorisonCoeff cd, double cf) {
        m_morison.push_back(std::make_unique<FrMorisonSingleElement_>(m_frame->GetBody(), frame, diameter,
                                                                      length, ca, cd, cf));
    }

    void FrMorisonCompositeElement_::SetDragCoeff(MorisonCoeff cd) {
        m_property.cd = cd;
    }

    void FrMorisonCompositeElement_::SetFrictionCoeff(double cf) {
        m_property.cf = cf;
    }

    void FrMorisonCompositeElement_::SetAddedMass(MorisonCoeff ca) {
        m_property.ca = ca;
    }

    void FrMorisonCompositeElement_::Initialize() {
        for (auto& element: m_morison) {
            element->Initialize();
        }
    }

    void FrMorisonCompositeElement_::Update(double time) {

        m_force.SetNull();
        m_torque.SetNull();

        for (auto& element : m_morison) {
            element->Update(time);
            m_force += element->GetForceInWorld(NWU);
            m_torque += element->GetTorqueInBody();
        }
    }

}
