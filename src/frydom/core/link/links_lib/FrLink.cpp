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

        void FrLinkLockBase::SetMask(FrBodyDOFMask* vmask) {
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

    FrLink::FrLink(std::shared_ptr<FrNode> node1, std::shared_ptr<FrNode> node2,
                     FrOffshoreSystem *system) :
                     FrLinkBase(node1, node2, system),
                     m_frame2WRT1_reference() {
        m_chronoLink = std::make_shared<internal::FrLinkLockBase>(this);
    }


    void FrLink::SetMarkers(FrNode* node1, FrNode* node2) {
        m_chronoLink->ReferenceMarkers(node1->m_chronoMarker.get(), node2->m_chronoMarker.get());
    }

    std::shared_ptr<chrono::ChLink> FrLink::GetChronoLink() {
        return m_chronoLink;
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

        auto torqueAtMarker1_ref = markerFrame_WRT_COG.ProjectVectorFrameInParent<Torque>(GetLinkReactionForceOnMarker1(fc), fc);
        auto COG_M1_ref = markerFrame_WRT_COG.GetPosition(fc);
        auto force_ref = markerFrame_WRT_COG.ProjectVectorFrameInParent<Force>(GetLinkReactionForceOnMarker1(fc), fc);

        return torqueAtMarker1_ref + COG_M1_ref.cross(force_ref);
    }

    const Torque FrLink::GetLinkReactionTorqueOnBody2AtCOG(FRAME_CONVENTION fc) const { // TODO : tester
        auto markerFrame_WRT_COG = m_node2->GetFrameWRT_COG_InBody();

        auto torqueAtMarker2_ref = markerFrame_WRT_COG.ProjectVectorFrameInParent<Torque>(GetLinkReactionForceOnMarker2(fc), fc);
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


    /*
     * FrBodyDOFMask definitions
     */

    FrBodyDOFMask::FrBodyDOFMask() = default;

    void FrBodyDOFMask::SetLock_X(bool lock) { m_xLocked = lock; }

    void FrBodyDOFMask::SetLock_Y(bool lock) { m_yLocked = lock; }

    void FrBodyDOFMask::SetLock_Z(bool lock) { m_zLocked = lock; }

    void FrBodyDOFMask::SetLock_Rx(bool lock) { m_RxLocked = lock; }

    void FrBodyDOFMask::SetLock_Ry(bool lock) { m_RyLocked = lock; }

    void FrBodyDOFMask::SetLock_Rz(bool lock) { m_RzLocked = lock; }

    void FrBodyDOFMask::LockXZPlane() {
        MakeItFree();
        SetLock_Y(true);
        SetLock_Rx(true);
        SetLock_Rz(true);
    }

    void FrBodyDOFMask::LockXYPlane() {
        MakeItFree();
        SetLock_Z(true);
        SetLock_Rx(true);
        SetLock_Ry(true);
    }

    bool FrBodyDOFMask::GetLock_X() const { return m_xLocked; }

    bool FrBodyDOFMask::GetLock_Y() const { return m_yLocked; }

    bool FrBodyDOFMask::GetLock_Z() const { return m_zLocked; }

    bool FrBodyDOFMask::GetLock_Rx() const { return m_RxLocked; }

    bool FrBodyDOFMask::GetLock_Ry() const { return m_RyLocked; }

    bool FrBodyDOFMask::GetLock_Rz() const { return m_RzLocked; }

    bool FrBodyDOFMask::HasLockedDOF() const {
        return m_xLocked || m_yLocked || m_zLocked || m_RxLocked || m_RyLocked || m_RzLocked;
    }

    bool FrBodyDOFMask::IsFree() const {
        return !HasLockedDOF();
    }

    void FrBodyDOFMask::MakeItFree() {
        m_xLocked = false;
        m_yLocked = false;
        m_zLocked = false;
        m_RxLocked = false;
        m_RyLocked = false;
        m_RzLocked = false;
    }

    void FrBodyDOFMask::MakeItLocked() {
        m_xLocked = true;
        m_yLocked = true;
        m_zLocked = true;
        m_RxLocked = true;
        m_RyLocked = true;
        m_RzLocked = true;
    }

    unsigned int FrBodyDOFMask::GetNbLockedDOF() const {
        unsigned int nb = 0;
        if (m_xLocked) nb++;
        if (m_yLocked) nb++;
        if (m_zLocked) nb++;
        if (m_RxLocked) nb++;
        if (m_RyLocked) nb++;
        if (m_RzLocked) nb++;
        return nb;
    }

    unsigned int FrBodyDOFMask::GetNbFreeDOF() const {
        return 6 - GetNbLockedDOF();
    }

    void FrLink::InitializeWithBodyDOFMask(FrBodyDOFMask *mask) {
        m_chronoLink->SetMask(mask);
    }

    FrFrame FrLink::GetConstraintViolation() const {
        return m_chronoLink->GetConstraintViolation();
    }

    void FrLink::UpdateCache() {}




}  // end namespace frydom
