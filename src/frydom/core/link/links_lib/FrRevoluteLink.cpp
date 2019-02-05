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
        UpdateCache();
    }

    double FrRevoluteLink::GetRestAngle() const {
        return m_restAngle;
    }

    const Direction FrRevoluteLink::GetLinkAxisInWorld(FRAME_CONVENTION fc) const {
        return GetNode1()->GetFrameInWorld().GetZAxisInParent(fc);
    }

    double FrRevoluteLink::GetLinkAngle() const {
        /*
         *
         * FIXME : le premier angle donne bien l'angle entre le marker 2 et le marker 1 mais ne compte pas les tours
         * Du coup, on a un angle qui se reinitialise lorsqu'on fait un tour et le modele d'effort se met a tirer dans
         * le mauvais sens !
         * Il convient de compter les tours et de permettre de recuperer un angle total, un angle entre 0 et pi et un
         * angle entre -pi et pi
         * Regarder comment est fait le motorLink de Chrono !!
         *
         * ChLinkMotorRotation::Update met en place le tracking !!! --> recopier la maniere de faire :)
         */

        return m_totalLinkAngle - GetRestAngle();
    }

    double FrRevoluteLink::GetLinkAngularVelocity() const {
        return m_linkAngularVelocity;
    }

    double FrRevoluteLink::GetLinkAngularAcceleration() const {
        return m_linkAngularAcceleration;
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

        // Update total angle measure (not on 0-2pi but continuous)
//        double lastTotalAngle = m_totalLinkAngle;
//        double lastRelativeAngle = remainder(lastTotalAngle, MU_2_PI);
//        double lastTurn = lastTotalAngle - lastRelativeAngle;
//
        auto currentRotation = GetMarker2OrientationWRTMarker1().GetRotationVector(NWU);
//
//
////        double newAngle = remainder(GetMarker2OrientationWRTMarker1().GetAngle(), MU_2_PI); // FIXME : ne serait-on pas plus secure si on ne prenait que la composante suicant z ?
//        // FIXME : cette partie ne fonctionne pas du tout !!!!!! --> En fait, on ne sait pas ce que renvoie le GetAngle() ci-dessus !!!
//
//        double newAngle = mathutils::Normalize_0_2PI(currentRotation[2]);
//
//
//
//        m_totalLinkAngle = lastTurn + newAngle;

        // Essai de virer ces 2 clauses qui foutent la merde...
//        if (fabs(newAngle + MU_2_PI - lastTotalAngle) < fabs(newAngle - lastTotalAngle))
//            m_totalLinkAngle = lastTurn + newAngle + MU_2_PI;
//        if (fabs(newAngle - MU_2_PI - lastTotalAngle) < fabs(newAngle - lastTotalAngle))
//            m_totalLinkAngle = lastTurn + newAngle - MU_2_PI;

//        std::cout << "Total angle = " << m_totalLinkAngle * RAD2DEG << std::endl;



        // TODO : voir a externaliser cet algo dans MathUtils !!! -> utile dans plein d'autres cas !!

        // TODO : compter les tours et les enregistrer ??

        // TODO : monitorer seulement l'angle fourni par le currentRelativeAngle --> voir s'il y a des sautes et quand

        double lastAngle = mathutils::Normalize__PI_PI(m_totalLinkAngle);
        double turnsAngle = remainder(lastAngle, MU_2_PI);

        double lastRelativeAngle = mathutils::Normalize__PI_PI(lastAngle - turnsAngle);

        double currentRelativeAngle =
                mathutils::Normalize__PI_PI(m_chronoLink->c_frame2WRT1.GetRotation().GetRotationVector(NWU)[2]);

        // TODO : voir a definir un RotationVector dans FrVector...
        double angleIncrement;
        if (fabs(currentRelativeAngle + MU_2_PI - lastRelativeAngle) < fabs(currentRelativeAngle - lastRelativeAngle)) {
            angleIncrement = currentRelativeAngle + MU_2_PI - lastRelativeAngle;
        } else if (fabs(currentRelativeAngle - MU_2_PI - lastRelativeAngle) < fabs(currentRelativeAngle - lastRelativeAngle)) {
            angleIncrement = currentRelativeAngle - MU_2_PI - lastRelativeAngle;
        } else {
            angleIncrement = currentRelativeAngle - lastRelativeAngle;
        }

        std::cout << "Angle increment : " << angleIncrement * RAD2DEG << std::endl;

        m_totalLinkAngle += angleIncrement;

        m_linkAngularVelocity = GetAngularVelocityOfMarker2WRTMarker1(NWU).GetWz();
        m_linkAngularAcceleration = GetAngularAccelerationOfMarker2WRTMarker1(NWU).GetWzp();

        UpdateForces(time);

    }

    void FrRevoluteLink::UpdateForces(double time) {
        Torque torque;
        torque.GetMz() = -m_stiffness * GetLinkAngle() - m_damping * GetLinkAngularVelocity();

        // TODO : voir si on applique pas un couple sur le mauvais axe dans le cas de contraintes mal resolues...
        SetLinkForceOnBody2InFrame2AtOrigin2(Force(), torque);  // TODO : verifier qu'on set la bonne chose...

        // TODO : si on a moteur force, on l'appelle ici et on ne prend pas en compte le spring damper...

    }

    void FrRevoluteLink::StepFinalize() {
        std::cout << "Total angle = " << GetLinkAngle() * RAD2DEG << std::endl;
        std::cout << "Torque = " << GetLinkTorqueOnBody2InFrame1AtOrigin2(NWU).GetMz() << std::endl;
        std::cout << "LinkVelocity = " << GetLinkAngularVelocity() << std::endl;

        std::cout << std::endl;
    }


    void FrRevoluteLink::MotorizeSpeed() {
        m_speedMotor = std::make_shared<internal::FrLinkMotorRotationSpeedBase>(this);
//        m_system->Add(m_speedMotor);
        // TODO : terminer

    }

    int FrRevoluteLink::GetNbTurns() const {
        return int(GetTurnAngle() / MU_2_PI);
    }

    double FrRevoluteLink::GetTurnAngle() const {
        return m_totalLinkAngle - remainder(m_totalLinkAngle, MU_2_PI);
    }



//    double FrRevoluteLink::GetAngleFromX() const {
//
//
//
//        GetMarker2OrientationWRTMarker1().GetAngle()
//
//
//
//
//
//    }


    void FrRevoluteLink::UpdateCache() {
        // Updating the rest angle
        m_restAngle = mathutils::Normalize__PI_PI(m_frame2WRT1_reference.GetRotation().GetAngle());
        // TODO : ne pas prendre GetAngle mais la composante z de RotationVector
    }


    std::shared_ptr<FrRevoluteLink>
    make_revolute_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_ *system) {
        auto link = std::make_shared<FrRevoluteLink>(node1, node2, system);
        system->AddLink(link);
        return link;
    }






}  // end namespace frydom
