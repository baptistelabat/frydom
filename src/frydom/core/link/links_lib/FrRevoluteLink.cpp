//
// Created by frongere on 23/01/19.
//

#include "FrRevoluteLink.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/core/common/FrNode.h"


namespace frydom {

    namespace internal {

        FrLinkMotorRotationSpeedBase::FrLinkMotorRotationSpeedBase(frydom::FrLink_ *frydomLink) : m_frydomLink(frydomLink) {
            SetSpindleConstraint(chrono::ChLinkMotorRotation::SpindleConstraint::FREE);
        }

        void FrLinkMotorRotationSpeedBase::Initialize() {

            // Based on ChLinkMateGeneric::Initialize

            this->Body1 = m_frydomLink->GetBody1()->GetChronoBody().get();
            this->Body2 = m_frydomLink->GetBody2()->GetChronoBody().get();

            this->mask->SetTwoBodiesVariables(&Body1->Variables(), &Body2->Variables());

            this->frame1 = internal::FrFrame2ChFrame(m_frydomLink->GetNode1()->GetFrameWRT_COG_InBody());
            this->frame2 = internal::FrFrame2ChFrame(m_frydomLink->GetNode2()->GetFrameWRT_COG_InBody());
        }

    }  // end namespace frydom::internal



    FrRevoluteLink::FrRevoluteLink(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2,
                                   FrOffshoreSystem_ *system) : FrLink_(node1, node2, system) {
        m_chronoLink->SetLinkType(REVOLUTE);
    }

    void FrRevoluteLink::SetSpringDamper(double stiffness, double damping) {
        m_stiffness = stiffness;
        m_damping = damping;
    }

    void FrRevoluteLink::SetRestAngle(double restAngle) {
        m_frame2WRT1_reference.SetRotZ_RADIANS(restAngle, NWU);
    }

    double FrRevoluteLink::GetRestAngle() const {
        return m_frame2WRT1_reference.GetRotation().GetAngle();
    }

    const Direction FrRevoluteLink::GetLinkAxisInWorld(FRAME_CONVENTION fc) const {
        return GetNode1()->GetFrameInWorld().GetZAxisInParent(fc);
    }

    double FrRevoluteLink::GetLinkAngle() const {
        return GetMarker2OrientationWRTMarker1().GetAngle() - GetRestAngle();
    }

    double FrRevoluteLink::GetLinkAngularVelocity() const {
        return GetAngularVelocityOfMarker2WRTMarker1(NWU).GetWz();
    }

    double FrRevoluteLink::GetLinkAngularAcceleration() const {
        return GetAngularAccelerationOfMarker2WRTMarker1(NWU).GetWzp();
    }

    void FrRevoluteLink::Initialize() {
        // Initialization of the constraint part
        FrLink_::Initialize();

        // Initialization of the motor part
        if (m_speedMotor) {
            m_speedMotor->Initialize();
        }

    }

    void FrRevoluteLink::Update(double time) {
        FrLink_::Update(time);

        Torque torque;
        torque.GetMz() = -m_stiffness * GetLinkAngle() - m_damping * GetLinkAngularVelocity();

        SetLinkForceOnBody2InFrame2AtOrigin2(Force(), torque);
    }

    void FrRevoluteLink::StepFinalize() {

    }


    void FrRevoluteLink::MotorizeSpeed() {
        m_speedMotor = std::make_shared<internal::FrLinkMotorRotationSpeedBase>(this);
        m_system->Add(m_speedMotor);
    }



    std::shared_ptr<FrRevoluteLink>
    make_revolute_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_ *system) {
        auto link = std::make_shared<FrRevoluteLink>(node1, node2, system);
        system->AddLink(link);
        return link;
    }




}  // end namespace frydom
