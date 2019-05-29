// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#include "FrRevoluteLink.h"

//#include "frydom/core/body/FrBody.h"
#include "frydom/core/common/FrNode.h"
#include "frydom/core/math/functions/FrFunctionsInc.h"

#include "frydom/core/link/links_lib/actuators/FrAngularActuator.h"


namespace frydom {

    FrRevoluteLink::FrRevoluteLink(const std::shared_ptr<FrNode>& node1, const std::shared_ptr<FrNode>& node2,
                                   FrOffshoreSystem *system) : FrLink(node1, node2, system) {
        m_chronoLink->SetLinkType(REVOLUTE);
    }

    void FrRevoluteLink::SetSpringDamper(double stiffness, double damping) {
        m_stiffness = stiffness;
        m_damping = damping;
    }

    void FrRevoluteLink::SetRestAngle(double restAngle) {
        m_frame2WRT1_reference.SetRotZ_RADIANS(restAngle, NWU);
        UpdateCache();
    }

    double FrRevoluteLink::GetRestAngle() const {
        return m_restAngle;
    }

    const Direction FrRevoluteLink::GetLinkAxisInWorld(FRAME_CONVENTION fc) const {
        return GetNode1()->GetFrameInWorld().GetZAxisInParent(fc);
    }

    double FrRevoluteLink::GetLinkAngle() const {
        return m_totalLinkAngle - m_restAngle;
    }

    double FrRevoluteLink::GetRelativeLinkAngle() const {
        return fmod(m_totalLinkAngle, MU_2PI) - m_restAngle;
    }

    int FrRevoluteLink::GetNbTurns() const {
        return int( (GetLinkAngle() - GetRelativeLinkAngle()) / MU_2PI );
    }

    double FrRevoluteLink::GetLinkAngularVelocity() const {
        return m_linkAngularVelocity;
    }

    double FrRevoluteLink::GetLinkAngularAcceleration() const {
        return m_linkAngularAcceleration;
    }

    double FrRevoluteLink::GetLinkTorque() const {
        return GetLinkTorqueOnBody2InFrame2AtOrigin2(NWU).GetMz();
    }

    double FrRevoluteLink::GetLinkPower() const {
        return GetLinkAngularVelocity() * GetLinkTorque();
    }

    void FrRevoluteLink::Initialize() {
        // Initialization of the constraint part
        FrLink::Initialize();

        // Initialization of the motor part
        if (m_actuator) {
            m_actuator->Initialize();
        }

    }

    void FrRevoluteLink::Update(double time) {

        FrLink::Update(time);

        double lastRelativeAngle = GetRelativeLinkAngle() + m_restAngle; // Making it relative to x, not the rest angle
        double updatedRelativeAngle = GetUpdatedRelativeAngle();

        // TODO : voir a definir un RotationVector dans FrVector...
        // Computing the angle increment between current relative angle and the last relative angle to increment
        // the total link angle
        double angleIncrement;
        if (fabs(updatedRelativeAngle + MU_2PI - lastRelativeAngle) < fabs(updatedRelativeAngle - lastRelativeAngle)) {
            angleIncrement = updatedRelativeAngle + MU_2PI - lastRelativeAngle;
        } else if (fabs(updatedRelativeAngle - MU_2PI - lastRelativeAngle) < fabs(updatedRelativeAngle - lastRelativeAngle)) {
            angleIncrement = updatedRelativeAngle - MU_2PI - lastRelativeAngle;
        } else {
            angleIncrement = updatedRelativeAngle - lastRelativeAngle;
        }

        m_totalLinkAngle += angleIncrement;

        m_linkAngularVelocity = GetAngularVelocityOfMarker2WRTMarker1(NWU).GetWz();
        m_linkAngularAcceleration = GetAngularAccelerationOfMarker2WRTMarker1(NWU).GetWzp();

        UpdateForces(time);

    }

    void FrRevoluteLink::UpdateForces(double time) {

        if (IsMotorized()) return;

        // Default spring damper force model
        Force force;
        Torque torque;

        torque.GetMz() = - m_stiffness * GetLinkAngle() - m_damping * GetLinkAngularVelocity();

        // Set the link force
        SetLinkForceTorqueOnBody2InFrame2AtOrigin2(force, torque);
    }

    FrAngularActuator *FrRevoluteLink::Motorize(ACTUATOR_CONTROL control) {
        m_actuator = std::make_shared<FrAngularActuator>(this, control);
        GetSystem()->Add(m_actuator);
        return dynamic_cast<FrAngularActuator*>(m_actuator.get());
    }

    double FrRevoluteLink::GetUpdatedRelativeAngle() const {
        return mathutils::Normalize__PI_PI(m_chronoLink->c_frame2WRT1.GetRotation().GetRotationVector(NWU)[2]);
//        return mathutils::Normalize__PI_PI(m_chronoLink->GetRelAngle()); // INFO : fonctionne bien moins bien que ci-dessus !
    }

    void FrRevoluteLink::UpdateCache() {
        // Updating the rest angle
        m_restAngle = mathutils::Normalize__PI_PI(m_frame2WRT1_reference.GetRotation().GetAngle());
        // TODO : ne pas prendre GetAngle mais la composante z de RotationVector

        // FIXME : attention si la liaison n'est pas resolue !!! Ca ne fonctionne pas
    }

    std::shared_ptr<FrRevoluteLink>
    make_revolute_link(std::shared_ptr<FrNode> node1, std::shared_ptr<FrNode> node2, FrOffshoreSystem *system) {
        auto link = std::make_shared<FrRevoluteLink>(node1, node2, system);
        system->AddLink(link);
        return link;
    }

    void FrRevoluteLink::Clamp() {

        if (IsMotorized()) GetSystem()->RemoveLink(m_actuator);

        // brake motorization instantiation
        m_actuator = std::make_shared<FrAngularActuator>(this, POSITION);
        m_actuator->Initialize();
        GetSystem()->Add(m_actuator);

        auto angle = GetMarker2OrientationWRTMarker1().GetAngle();

        m_actuator->SetMotorFunction(FrConstantFunction(angle));

    }



}  // end namespace frydom
