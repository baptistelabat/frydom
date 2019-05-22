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


#include "FrLink.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"
#include "actuators/FrActuator.h"


namespace frydom {

    namespace internal {

        FrLinkLockBase::FrLinkLockBase(frydom::FrLink *frydomLink) : m_frydomLink(frydomLink), chrono::ChLinkLock() {}

        void FrLinkLockBase::SetLinkType(LINK_TYPE lt) {
            switch (lt) {
                case CYLINDRICAL:
                    ChangeLinkType(ChronoLinkType::CYLINDRICAL);
                    break;
                case FIXED_LINK:
                    ChangeLinkType(ChronoLinkType::LOCK);
                    break;
                case FREE_LINK:
                    ChangeLinkType(ChronoLinkType::FREE);
                    break;
                case PRISMATIC:
                    ChangeLinkType(ChronoLinkType::PRISMATIC);
                    break;
                case REVOLUTE:
                    ChangeLinkType(ChronoLinkType::REVOLUTE);
                    break;
//                    case SCREW:
//                        ChangeLinkType(ChronoLinkType::CYLINDRICAL);
//                        break;
                case SPHERICAL:
                    ChangeLinkType(ChronoLinkType::SPHERICAL);
                    break;
                case PERPENDICULAR:
                    ChangeLinkType(ChronoLinkType::PERPEND);
                    break;
                case PARALLEL:
                    ChangeLinkType(ChronoLinkType::PARALLEL);
                    break;
                case PLANEONPLANE:
                    ChangeLinkType(ChronoLinkType::PLANEPLANE);
                    break;
//                case DISTANCETOAXIS:
//                    ChangeLinkType(ChronoLinkType::);
//                    break;
                case POINTONLINE:
                    ChangeLinkType(ChronoLinkType::POINTLINE);
                    break;
                case POINTONPLANE:
                    ChangeLinkType(ChronoLinkType::POINTPLANE);
                    break;
//                case POINTONSPLINE:
//                    ChangeLinkType(ChronoLinkType::);
//                    break;
            }
        }

        void FrLinkLockBase::SetupInitial() {
            m_frydomLink->Initialize();
        }

        void FrLinkLockBase::Update(double time, bool update_assets) {

            chrono::ChLinkLock::Update(time, update_assets);

            GenerateCache();

            m_frydomLink->Update(time);

        }

        void FrLinkLockBase::GenerateCache() {
            // 1 - Relative Frames
            c_frame1WRT2 = internal::ChCoordsys2FrFrame(GetRelM());
            c_frame2WRT1 = c_frame1WRT2.GetInverse();

            // 2 - Relative velocities
            c_generalizedVelocity1WRT2.SetVelocity(internal::ChVectorToVector3d<Velocity>(GetRelM_dt().pos));
            c_generalizedVelocity1WRT2.SetAngularVelocity(internal::ChVectorToVector3d<AngularVelocity>(GetRelWvel()));

            c_generalizedVelocity2WRT1.SetVelocity(
                    - c_frame2WRT1.ProjectVectorFrameInParent<Velocity>(c_generalizedVelocity1WRT2.GetVelocity(), NWU));
            c_generalizedVelocity2WRT1.SetAngularVelocity(
                    - c_frame2WRT1.ProjectVectorFrameInParent<AngularVelocity>(c_generalizedVelocity1WRT2.GetAngularVelocity(), NWU));

            // 2 - Relative accelerations
            c_generalizedAcceleration1WRT2.SetAcceleration(internal::ChVectorToVector3d<Acceleration>(GetRelM_dtdt().pos));
            c_generalizedAcceleration1WRT2.SetAngularAcceleration(internal::ChVectorToVector3d<AngularAcceleration>(GetRelWacc()));

            c_generalizedAcceleration2WRT1.SetAcceleration(
                    - c_frame2WRT1.ProjectVectorFrameInParent<Acceleration>(c_generalizedAcceleration1WRT2.GetAcceleration(), NWU));
            c_generalizedAcceleration2WRT1.SetAngularAcceleration(
                    - c_frame2WRT1.ProjectVectorFrameInParent<AngularAcceleration>(c_generalizedAcceleration1WRT2.GetAngularAcceleration(), NWU));

            // 3 - Link forces
            c_generalizedForceOnMarker2.SetForce(internal::ChVectorToVector3d<Force>(Get_react_force()));
            c_generalizedForceOnMarker2.SetTorque(internal::ChVectorToVector3d<Torque>(Get_react_torque()));

            c_generalizedForceOnMarker1.SetForce(
                    - c_frame2WRT1.ProjectVectorFrameInParent<Force>(c_generalizedForceOnMarker2.GetForce(), NWU));
            c_generalizedForceOnMarker1.SetTorque(
                    - c_frame2WRT1.ProjectVectorFrameInParent(c_generalizedForceOnMarker2.GetTorque(), NWU)
                    + c_frame2WRT1.GetPosition(NWU).cross(c_generalizedForceOnMarker1.GetForce())
                    );

        }

        void FrLinkLockBase::SetMask(FrDOFMask* vmask) {

            if (vmask->GetLinkType() == LINK_TYPE::CUSTOM) {
                chrono::ChLinkMaskLF chronoMask;
                chronoMask.SetLockMask(
                    vmask->GetLock_X(),  // x
                    vmask->GetLock_Y(),  // y
                    vmask->GetLock_Z(),  // z
                    false,              // e0
                    vmask->GetLock_Rx(), // e1
                    vmask->GetLock_Ry(), // e2
                    vmask->GetLock_Rz()  // e3
                );
                BuildLink(&chronoMask);
            } else {
                this->SetLinkType(vmask->GetLinkType());
            }

        }

        void FrLinkLockBase::SetLinkForceOnBody1InFrame2AtOrigin1(const Force &force) {
            C_force = internal::Vector3dToChVector(force);
        }

        void FrLinkLockBase::SetLinkTorqueOnBody1InFrame2AtOrigin1(const Torque& torque) {
            C_torque = internal::Vector3dToChVector(torque);
        }

        Force FrLinkLockBase::GetLinkForceOnBody1InFrame2AtOrigin1() {
            return internal::ChVectorToVector3d<Force>(C_force);
        }

        Torque FrLinkLockBase::GetLinkTorqueOnBody1InFrame2ArOrigin1() {
            return internal::ChVectorToVector3d<Torque>(C_torque);
        }

        FrFrame FrLinkLockBase::GetConstraintViolation() { // TODO : voir si c'est bien la violation de 2 par rapport a 1, dans 1 !! sinon, renvoyer l'inverse
            return internal::ChCoordsys2FrFrame(GetRelC());
        }

    }  // end namespace frydom::internal


    /*
     * FrLink method definitions
     *
     */

    FrLink::FrLink(const std::shared_ptr<FrNode>& node1, const std::shared_ptr<FrNode>& node2,
                     FrOffshoreSystem *system) :
                     FrLinkBase(node1, node2, system),
                     m_frame2WRT1_reference() {
        m_chronoLink = std::make_shared<internal::FrLinkLockBase>(this);
        SetLogged(true);
        m_actuator = nullptr;
    }


    void FrLink::SetMarkers(FrNode* node1, FrNode* node2) {
        // IMPORTANT : in FRyDoM the first node is the master and the second one the slave, as opposed to Chrono !!!
        m_chronoLink->ReferenceMarkers(node2->m_chronoMarker.get(), node1->m_chronoMarker.get());
    }

    std::shared_ptr<chrono::ChLink> FrLink::GetChronoLink() {
        return m_chronoLink;
    }

    internal::FrLinkLockBase *FrLink::GetChronoItem_ptr() const {
        return m_chronoLink.get();
    }


    bool FrLink::IsDisabled() const {
        return m_chronoLink->IsDisabled();
    }

    void FrLink::SetDisabled(bool disabled) {
        m_chronoLink->SetDisabled(disabled);
        if (IsMotorized()) {
            m_actuator->SetDisabled(disabled);
        }
    }

    void FrLink::SetBreakable(bool breakable) {
        m_breakable = breakable;
    }

    bool FrLink::IsBreakable() const {
        return m_breakable;
    }

    bool FrLink::IsBroken() const {
        return m_chronoLink->IsBroken();
    }

    void FrLink::SetBroken(bool broken) {
        if (!IsBreakable()) return;

        m_chronoLink->SetBroken(broken);
        if (IsMotorized()) {
            m_actuator->SetDisabled(broken);
        }
    }

    void FrLink::SetLocked(bool locked) {
        if (locked) {
            m_chronoLink->SetLinkType(FIXED_LINK);
        } else {
            m_chronoLink->SetMask(&m_dofMask);
        }
    }

    bool FrLink::IsActive() const {
        return m_chronoLink->IsActive();
    }

    bool FrLink::IsMotorized() const {
        return (m_actuator && m_actuator->IsActive());
    }

    void FrLink::SetThisConfigurationAsReference() {
        m_frame2WRT1_reference = m_chronoLink->c_frame2WRT1;  // FIXME : cette methode devrait trigger l'update des caches de classes derivees
        UpdateCache();
    }

    // Must be reimplemented in
//    bool FrLink::IsMotorized() const { return false; }

    const FrFrame FrLink::GetMarker2FrameWRTMarker1Frame() const {
        return m_chronoLink->c_frame2WRT1;
    }

    const FrFrame FrLink::GetMarker1FrameWRTMarker2Frame() const {
        return m_chronoLink->c_frame1WRT2;
    }

    const Position FrLink::GetMarker2PositionWRTMarker1(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_frame2WRT1.GetPosition(fc);
    }

    const Position FrLink::GetMarker1PositionWRTMarker2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_frame1WRT2.GetPosition(fc);
    }

    const FrRotation FrLink::GetMarker2OrientationWRTMarker1() const {
        return m_chronoLink->c_frame2WRT1.GetRotation();
    }

    const FrRotation FrLink::GetMarker1OrientationWRTMarker2() const {
        return m_chronoLink->c_frame1WRT2.GetRotation();
    }

    const GeneralizedVelocity FrLink::GetGeneralizedVelocityOfMarker2WRTMarker1(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedVelocity2WRT1;
    }

    const GeneralizedVelocity FrLink::GetGeneralizedVelocityOfMarker1WRTMarker2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedVelocity1WRT2;
    }

    const Velocity FrLink::GetVelocityOfMarker2WRTMarker1(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedVelocity2WRT1.GetVelocity();
    }

    const Velocity FrLink::GetVelocityOfMarker1WRTMarker2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedVelocity1WRT2.GetVelocity();
    }

    const AngularVelocity FrLink::GetAngularVelocityOfMarker2WRTMarker1(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedVelocity2WRT1.GetAngularVelocity();
    }

    const AngularVelocity FrLink::GetAngularVelocityOfMarker1WRTMarker2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedVelocity1WRT2.GetAngularVelocity();
    }

    const GeneralizedAcceleration FrLink::GetGeneralizedAccelerationOfMarker2WRTMarker1(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedAcceleration2WRT1;
    }

    const GeneralizedAcceleration FrLink::GetGeneralizedAccelerationOfMarker1WRTMarker2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedAcceleration1WRT2;
    }

    const Acceleration FrLink::GetAccelerationOfMarker1WRTMarker2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedAcceleration1WRT2.GetAcceleration();
    }

    const Acceleration FrLink::GetAccelerationOfMarker2WRTMarker1(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedAcceleration2WRT1.GetAcceleration();
    }

    const AngularAcceleration FrLink::GetAngularAccelerationOfMarker2WRTMarker1(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedAcceleration2WRT1.GetAngularAcceleration();
    }

    const AngularAcceleration FrLink::GetAngularAccelerationOfMarker1WRTMarker2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedAcceleration1WRT2.GetAngularAcceleration();
    }

    const Force FrLink::GetLinkReactionForceOnMarker1(FRAME_CONVENTION fc) const { // TODO : tester
        auto force = m_chronoLink->c_generalizedForceOnMarker1.GetForce();
        if (IsNED(fc)) internal::SwapFrameConvention<Force>(force);
        return force;
    }

    const Force FrLink::GetLinkReactionForceOnMarker2(FRAME_CONVENTION fc) const { // TODO : tester
        auto force = m_chronoLink->c_generalizedForceOnMarker2.GetForce();
        if (IsNED(fc)) internal::SwapFrameConvention<Force>(force);
        return force;
    }

    const Force FrLink::GetLinkReactionForceOnBody1(FRAME_CONVENTION fc) const { // TODO : tester
        auto forceOnMarker1 = GetLinkReactionForceOnMarker1(fc);
        return m_node1->GetFrameWRT_COG_InBody().ProjectVectorFrameInParent(forceOnMarker1, fc);
    }

    const Force FrLink::GetLinkReactionForceOnBody2(FRAME_CONVENTION fc) const { // TODO : tester
        auto forceOnMarker2 = GetLinkReactionForceOnMarker2(fc);
        return m_node2->GetFrameWRT_COG_InBody().ProjectVectorFrameInParent(forceOnMarker2, fc);
    }

    const Torque FrLink::GetLinkReactionTorqueOnMarker1(FRAME_CONVENTION fc) const { // TODO : tester
        auto torque = m_chronoLink->c_generalizedForceOnMarker1.GetTorque();
        if (IsNED(fc)) internal::SwapFrameConvention<Torque>(torque);
        return torque;
    }

    const Torque FrLink::GetLinkReactionTorqueOnMarker2(FRAME_CONVENTION fc) const { // TODO : tester
        auto torque = m_chronoLink->c_generalizedForceOnMarker2.GetTorque();
        if (IsNED(fc)) internal::SwapFrameConvention<Torque>(torque);
        return torque;
    }

    const Torque FrLink::GetLinkReactionTorqueOnBody1AtCOG(FRAME_CONVENTION fc) const { // TODO : tester
        auto markerFrame_WRT_COG = m_node1->GetFrameWRT_COG_InBody();

        auto torqueAtMarker1_ref = markerFrame_WRT_COG.ProjectVectorFrameInParent<Torque>(GetLinkReactionTorqueOnMarker1(fc), fc);
        auto COG_M1_ref = markerFrame_WRT_COG.GetPosition(fc);
        auto force_ref = markerFrame_WRT_COG.ProjectVectorFrameInParent<Force>(GetLinkReactionForceOnMarker1(fc), fc);

        return torqueAtMarker1_ref + COG_M1_ref.cross(force_ref);
    }

    const Torque FrLink::GetLinkReactionTorqueOnBody2AtCOG(FRAME_CONVENTION fc) const { // TODO : tester
        auto markerFrame_WRT_COG = m_node2->GetFrameWRT_COG_InBody();

        auto torqueAtMarker2_ref = markerFrame_WRT_COG.ProjectVectorFrameInParent<Torque>(GetLinkReactionTorqueOnMarker2(fc), fc);
        auto COG_M2_ref = markerFrame_WRT_COG.GetPosition(fc);
        auto force_ref = markerFrame_WRT_COG.ProjectVectorFrameInParent<Force>(GetLinkReactionForceOnMarker2(fc), fc);

        return torqueAtMarker2_ref + COG_M2_ref.cross(force_ref);
    }

    void FrLink::Initialize() {

        SetMarkers(m_node1.get(), m_node2.get());
    }

    void FrLink::Update(double time) {
        // TODO : Ici, on met en cache les differentes quantites utiles a FrLink mise en convention frydom ie
        // les donnes de la liaison sont relatives a un mouvement du node 2 par rapport au node 1
        // On fait appel aux methodes de FrLinkLockBase pour faciliter
        // Du coup, l'update de l'objet Chrono doit etre fait avant l'objet frydom

    }

    void FrLink::SetLinkForceTorqueOnBody2InFrame2AtOrigin2(const Force &force, const Torque &torque) {
        /* From Chrono comments in ChLinkMasked::UpdateForces :
         * C_force and C_torque   are considered in the reference coordsystem
         * of marker2  (the MAIN marker), and their application point is considered the
         * origin of marker1 (the SLAVE marker)
         */

        // Force
        m_chronoLink->SetLinkForceOnBody1InFrame2AtOrigin1(-force); // The minus signe is to get the force applied on body 1

        // Torque transport
        auto O1O2 = -m_chronoLink->c_frame1WRT2.GetPosition(NWU);

        Torque torque_M1 = -torque + O1O2.cross(-force);
        m_chronoLink->SetLinkTorqueOnBody1InFrame2AtOrigin1(torque_M1);
    }

    const Force FrLink::GetLinkForceOnBody1InFrame1AtOrigin1(FRAME_CONVENTION fc) const {
        auto force = m_chronoLink->GetLinkForceOnBody1InFrame2AtOrigin1();
        return m_chronoLink->c_frame2WRT1.ProjectVectorFrameInParent<Force>(force, fc);
    }

    const Force FrLink::GetLinkForceOnBody2InFrame2AtOrigin2(FRAME_CONVENTION fc) const {
        return - m_chronoLink->GetLinkForceOnBody1InFrame2AtOrigin1();
    }

    const Torque FrLink::GetLinkTorqueOnBody1InFrame1AtOrigin1(FRAME_CONVENTION fc) const {
        auto torque = m_chronoLink->GetLinkTorqueOnBody1InFrame2ArOrigin1();
        return m_chronoLink->c_frame2WRT1.ProjectVectorFrameInParent<Force>(torque, fc);
    }

    const Torque FrLink::GetLinkTorqueOnBody2InFrame2AtOrigin2(FRAME_CONVENTION fc) const {
        auto torque_O1_2 = m_chronoLink->GetLinkTorqueOnBody1InFrame2ArOrigin1();
        auto force_2 = m_chronoLink->GetLinkForceOnBody1InFrame2AtOrigin1();
        auto O2O1_2 = m_chronoLink->c_frame1WRT2.GetPosition(NWU);
        return -(torque_O1_2 + O2O1_2.cross(force_2));
    }

    const Force FrLink::GetLinkForceOnBody1InFrame2AtOrigin1(FRAME_CONVENTION fc) const {
        return m_chronoLink->GetLinkForceOnBody1InFrame2AtOrigin1();
    }

    const Force FrLink::GetLinkForceOnBody2InFrame1AtOrigin2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_frame2WRT1.ProjectVectorFrameInParent<Force>(GetLinkForceOnBody2InFrame2AtOrigin2(fc), fc);
    }

    const Torque FrLink::GetLinkTorqueOnBody1InFrame2AtOrigin1(FRAME_CONVENTION fc) const {
        return m_chronoLink->GetLinkTorqueOnBody1InFrame2ArOrigin1();
    }

    const Torque FrLink::GetLinkTorqueOnBody2InFrame1AtOrigin2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_frame2WRT1.ProjectVectorFrameInParent<Torque>(GetLinkTorqueOnBody2InFrame2AtOrigin2(fc), fc);
    }


    double FrLink::GetLinkPower() const {
        return GetLinkForceOnBody2InFrame1AtOrigin2(NWU).dot(GetVelocityOfMarker2WRTMarker1(NWU))
             + GetLinkTorqueOnBody2InFrame1AtOrigin2(NWU).dot(GetAngularVelocityOfMarker2WRTMarker1(NWU));
    }

    void FrLink::AddFields() {
        m_message->AddField<double>("time", "s", "Current time of the simulation",
                                    [this]() { return m_system->GetTime(); });

        // Marker Position
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("Marker2PositionWRTMarker1","m", fmt::format("Marker 2 position relatively to Marker 1, in Marker 1 reference frame in {}", GetLogFrameConvention()),
                 [this]() {return GetMarker2PositionWRTMarker1(GetLogFrameConvention());});
        // Marker Velocity
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("VelocityOfMarker2WRTMarker1","m/s", fmt::format("Marker 2 velocity relatively to Marker 1, in Marker 1 reference frame in {}", GetLogFrameConvention()),
                 [this]() {return GetVelocityOfMarker2WRTMarker1(GetLogFrameConvention());});
        // Marker Acceleration
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("AccelerationOfMarker2WRTMarker1","m/s^2", fmt::format("Marker 2 acceleration relatively to Marker 1, in Marker 1 reference frame in {}", GetLogFrameConvention()),
                 [this]() {return GetAccelerationOfMarker2WRTMarker1(GetLogFrameConvention());});

        // Marker Position
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("Marker2OrientationWRTMarker1","rad", fmt::format("Marker 2 orientation relatively to Marker 1, in Marker 1 reference frame in {}", GetLogFrameConvention()),
                 [this]() {double phi, theta, psi; GetMarker2OrientationWRTMarker1().GetCardanAngles_RADIANS(phi, theta, psi, GetLogFrameConvention());
                    return Position(phi, theta, psi);});
        // Marker Velocity
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("AngularVelocityOfMarker2WRTMarker1","rad/s", fmt::format("Marker 2 angular velocity relatively to Marker 1, in Marker 1 reference frame in {}", GetLogFrameConvention()),
                 [this]() {return GetAngularVelocityOfMarker2WRTMarker1(GetLogFrameConvention());});
        // Marker Acceleration
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("AngularAccelerationOfMarker2WRTMarker1","m/s^2", fmt::format("Marker 2 angular acceleration relatively to Marker 1, in Marker 1 reference frame in {}", GetLogFrameConvention()),
                 [this]() {return GetAngularAccelerationOfMarker2WRTMarker1(GetLogFrameConvention());});


        // Force
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("LinkReactionForceOnBody1","N", fmt::format("link reaction force applied at marker 1, expressed in body 1 reference frame in {}", GetLogFrameConvention()),
                 [this]() {return GetLinkReactionForceOnBody1(GetLogFrameConvention());});
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("LinkReactionForceOnBody2","N", fmt::format("link reaction force applied at marker 2, expressed in body 2 reference frame in {}", GetLogFrameConvention()),
                 [this]() {return GetLinkReactionForceOnBody2(GetLogFrameConvention());});
        // Torque
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("LinkReactionTorqueOnBody1AtCOG","Nm", fmt::format("link reaction torque at CoG applied at marker 1, expressed in body 1 reference frame in {}", GetLogFrameConvention()),
                 [this]() {return GetLinkReactionTorqueOnBody1AtCOG(GetLogFrameConvention());});
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("LinkReactionTorqueOnBody2AtCOG","Nm", fmt::format("link reaction torque at CoG applied at marker 2, expressed in body 2 reference frame in {}", GetLogFrameConvention()),
                 [this]() {return GetLinkReactionTorqueOnBody2AtCOG(GetLogFrameConvention());});

        // Power
        m_message->AddField<double>
                ("LinkPower","kW", "power delivered in a FrLink", [this]() {return 0.001*GetLinkPower();});

        // Motor
        if (m_actuator) {
            m_message->AddField<double>
                    ("MotorPower","kW", "power delivered by the motor", [this]() {return m_actuator->GetMotorPower();});
            m_message->AddField<Eigen::Matrix<double, 3, 1>>
                    ("MotorForceInBody1","N", fmt::format("Force applied by the motor on body 1, in body 1 reference frame {}", GetLogFrameConvention()),
                     [this]() {return m_actuator->GetMotorForceInBody1(GetLogFrameConvention());});
            m_message->AddField<Eigen::Matrix<double, 3, 1>>
                    ("MotorForceInBody2","N", fmt::format("Force applied by the motor on body 1, in body 2 reference frame {}", GetLogFrameConvention()),
                     [this]() {return m_actuator->GetMotorForceInBody2(GetLogFrameConvention());});
            m_message->AddField<Eigen::Matrix<double, 3, 1>>
                    ("MotorTorqueAtCOGInBody1(","Nm", fmt::format("Torque applied by the motor at COG on body 1, in body 1 reference frame {}", GetLogFrameConvention()),
                     [this]() {return m_actuator->GetMotorTorqueAtCOGInBody1(GetLogFrameConvention());});
            m_message->AddField<Eigen::Matrix<double, 3, 1>>
                    ("MotorTorqueAtCOGInBody2(","Nm", fmt::format("Torque applied by the motor at COG on body 2, in body 2 reference frame {}", GetLogFrameConvention()),
                     [this]() {return m_actuator->GetMotorTorqueAtCOGInBody2(GetLogFrameConvention());});

        }

    }

    void FrLink::InitializeWithBodyDOFMask(FrDOFMask *mask) {
        m_chronoLink->SetMask(mask);
    }

    FrFrame FrLink::GetConstraintViolation() const {
        return m_chronoLink->GetConstraintViolation();
    }

    void FrLink::UpdateCache() {}

    /*
     * FrDOFMask definitions
     */

    FrDOFMask::FrDOFMask() = default;

    void FrDOFMask::SetLock_X(bool lock) {
        m_xLocked = lock;
        m_linkType = LINK_TYPE::CUSTOM;
    }

    void FrDOFMask::SetLock_Y(bool lock) {
        m_yLocked = lock;
        m_linkType = LINK_TYPE::CUSTOM;
    }

    void FrDOFMask::SetLock_Z(bool lock) {
        m_zLocked = lock;
        m_linkType = LINK_TYPE::CUSTOM;
    }

    void FrDOFMask::SetLock_Rx(bool lock) {
        m_RxLocked = lock;
        m_linkType = LINK_TYPE::CUSTOM;
    }

    void FrDOFMask::SetLock_Ry(bool lock) {
        m_RyLocked = lock;
        m_linkType = LINK_TYPE::CUSTOM;
    }

    void FrDOFMask::SetLock_Rz(bool lock) {
        m_RzLocked = lock;
        m_linkType = LINK_TYPE::CUSTOM;
    }

    void FrDOFMask::LockXZPlane() {
        MakeItFree();
        SetLock_Y(true);
        SetLock_Rx(true);
        SetLock_Rz(true);
    }

    void FrDOFMask::LockXYPlane() {
        MakeItFree();
        SetLock_Z(true);
        SetLock_Rx(true);
        SetLock_Ry(true);
    }

    bool FrDOFMask::GetLock_X() const { return m_xLocked; }

    bool FrDOFMask::GetLock_Y() const { return m_yLocked; }

    bool FrDOFMask::GetLock_Z() const { return m_zLocked; }

    bool FrDOFMask::GetLock_Rx() const { return m_RxLocked; }

    bool FrDOFMask::GetLock_Ry() const { return m_RyLocked; }

    bool FrDOFMask::GetLock_Rz() const { return m_RzLocked; }

    bool FrDOFMask::HasLockedDOF() const {
        return m_xLocked || m_yLocked || m_zLocked || m_RxLocked || m_RyLocked || m_RzLocked;
    }

    bool FrDOFMask::IsFree() const {
        return !HasLockedDOF();
    }

    void FrDOFMask::MakeItFree() {
        m_xLocked = false;
        m_yLocked = false;
        m_zLocked = false;
        m_RxLocked = false;
        m_RyLocked = false;
        m_RzLocked = false;
        m_linkType = LINK_TYPE::FREE_LINK;
    }

    void FrDOFMask::MakeItLocked() {
        m_xLocked = true;
        m_yLocked = true;
        m_zLocked = true;
        m_RxLocked = true;
        m_RyLocked = true;
        m_RzLocked = true;
        m_linkType = LINK_TYPE::FIXED_LINK;
    }

    unsigned int FrDOFMask::GetNbLockedDOF() const {
        unsigned int nb = 0;
        if (m_xLocked) nb++;
        if (m_yLocked) nb++;
        if (m_zLocked) nb++;
        if (m_RxLocked) nb++;
        if (m_RyLocked) nb++;
        if (m_RzLocked) nb++;
        return nb;
    }

    unsigned int FrDOFMask::GetNbFreeDOF() const {
        return 6 - GetNbLockedDOF();
    }

    void FrDOFMask::SetLinkType(frydom::LINK_TYPE linkType) {
        m_linkType = linkType;

        switch(m_linkType) {
            case LINK_TYPE::FREE_LINK:
                SetLock(false, false, false, false, false, false);
                break;
            case LINK_TYPE::FIXED_LINK:
                SetLock(true, true, true, true, true, true);
                break;
            case LINK_TYPE::REVOLUTE:
                SetLock(true, true, true, true, true, false);
                break;
            case LINK_TYPE::PRISMATIC:
                SetLock(true, true, false, true, true, true);
                break;
            case LINK_TYPE::CYLINDRICAL:
                SetLock(true, true, false, true, true, false);
                break;
            case LINK_TYPE::SPHERICAL:
                SetLock(true, true, true, false ,false, false);
                break;
            default:
                SetLock(false, false, false, false, false, false);
                break;
        }
    }

    LINK_TYPE FrDOFMask::GetLinkType() const {
        return m_linkType;
    }

    void FrDOFMask::SetLock(bool xLocked, bool yLocked, bool zLocked, bool rxLocked, bool ryLocked, bool rzLocked) {
        m_xLocked = xLocked;
        m_yLocked = yLocked;
        m_zLocked = zLocked;
        m_RxLocked = rxLocked;
        m_RyLocked = ryLocked;
        m_RzLocked = rzLocked;
    }


}  // end namespace frydom
