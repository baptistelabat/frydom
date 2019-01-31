//
// Created by frongere on 20/09/18.
//

#include "FrLink.h"

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMotor.h"

#include <frydom/core/common/FrNode.h>





namespace frydom {

    namespace internal {

        FrLinkLockBase::FrLinkLockBase(frydom::FrLink_ *frydomLink) : m_frydomLink(frydomLink), chrono::ChLinkLock() {}

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

    }  // end namespace frydom::internal


    /*
     * FrLink_ method definitions
     *
     */

    FrLink_::FrLink_(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2,
                     FrOffshoreSystem_ *system) :
                     FrLinkBase_(node1, node2, system),
                     m_frame2WRT1_reference() {
        m_chronoLink = std::make_shared<internal::FrLinkLockBase>(this);
    }


    void FrLink_::SetMarkers(FrNode_* node1, FrNode_* node2) {
        m_chronoLink->SetUpMarkers(node1->m_chronoMarker.get(), node2->m_chronoMarker.get());
    }

    std::shared_ptr<chrono::ChLink> FrLink_::GetChronoLink() {
        return m_chronoLink;
    }

    bool FrLink_::IsDisabled() const {
        return m_chronoLink->IsDisabled();
    }

    void FrLink_::SetDisabled(bool disabled) {
        m_chronoLink->SetDisabled(disabled);
    }

    bool FrLink_::IsBroken() const {
        return m_chronoLink->IsBroken();
    }

    void FrLink_::SetBroken(bool broken) {
        m_chronoLink->SetBroken(broken);
    }

    bool FrLink_::IsActive() const {
        return m_chronoLink->IsActive();
    }

    void FrLink_::SetThisConfigurationAsReference() {
        m_frame2WRT1_reference = m_chronoLink->c_frame2WRT1;
    }


    const FrFrame_ FrLink_::GetMarker2FrameWRTMarker1Frame() const {
        return m_chronoLink->c_frame2WRT1;
    }

    const FrFrame_ FrLink_::GetMarker1FrameWRTMarker2Frame() const {
        return m_chronoLink->c_frame1WRT2;
    }

    const Position FrLink_::GetMarker2PositionWRTMarker1(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_frame2WRT1.GetPosition(fc);
    }

    const Position FrLink_::GetMarker1PositionWRTMarker2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_frame1WRT2.GetPosition(fc);
    }

    const FrRotation_ FrLink_::GetMarker2OrientationWRTMarker1() const {
        return m_chronoLink->c_frame2WRT1.GetRotation();
    }

    const FrRotation_ FrLink_::GetMarker1OrientationWRTMarker2() const {
        return m_chronoLink->c_frame1WRT2.GetRotation();
    }

    const GeneralizedVelocity FrLink_::GetGeneralizedVelocityOfMarker2WRTMarker1(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedVelocity2WRT1;
    }

    const GeneralizedVelocity FrLink_::GetGeneralizedVelocityOfMarker1WRTMarker2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedVelocity1WRT2;
    }

    const Velocity FrLink_::GetVelocityOfMarker2WRTMarker1(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedVelocity2WRT1.GetVelocity();
    }

    const Velocity FrLink_::GetVelocityOfMarker1WRTMarker2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedVelocity1WRT2.GetVelocity();
    }

    const AngularVelocity FrLink_::GetAngularVelocityOfMarker2WRTMarker1(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedVelocity2WRT1.GetAngularVelocity();
    }

    const AngularVelocity FrLink_::GetAngularVelocityOfMarker1WRTMarker2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedVelocity1WRT2.GetAngularVelocity();
    }

    const GeneralizedAcceleration FrLink_::GetGeneralizedAccelerationOfMarker2WRTMarker1(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedAcceleration2WRT1;
    }

    const GeneralizedAcceleration FrLink_::GetGeneralizedAccelerationOfMarker1WRTMarker2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedAcceleration1WRT2;
    }

    const Acceleration FrLink_::GetAccelerationOfMarker1WRTMarker2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedAcceleration1WRT2.GetAcceleration();
    }

    const Acceleration FrLink_::GetAccelerationOfMarker2WRTMarker1(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedAcceleration2WRT1.GetAcceleration();
    }

    const AngularAcceleration FrLink_::GetAngularAccelerationOfMarker2WRTMarker1(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedAcceleration2WRT1.GetAngularAcceleration();
    }

    const AngularAcceleration FrLink_::GetAngularAccelerationOfMarker1WRTMarker2(FRAME_CONVENTION fc) const {
        return m_chronoLink->c_generalizedAcceleration1WRT2.GetAngularAcceleration();
    }

    const Force FrLink_::GetLinkReactionForceOnMarker1(FRAME_CONVENTION fc) const { // TODO : tester
        auto force = m_chronoLink->c_generalizedForceOnMarker1.GetForce();
        if (IsNED(fc)) internal::SwapFrameConvention<Force>(force);
        return force;
    }

    const Force FrLink_::GetLinkReactionForceOnMarker2(FRAME_CONVENTION fc) const { // TODO : tester
        auto force = m_chronoLink->c_generalizedForceOnMarker2.GetForce();
        if (IsNED(fc)) internal::SwapFrameConvention<Force>(force);
        return force;
    }

    const Force FrLink_::GetLinkReactionForceOnBody1(FRAME_CONVENTION fc) const { // TODO : tester
        auto forceOnMarker1 = GetLinkReactionForceOnMarker1(fc);
        return m_node1->GetFrameWRT_COG_InBody().ProjectVectorFrameInParent(forceOnMarker1, fc);
    }

    const Force FrLink_::GetLinkReactionForceOnBody2(FRAME_CONVENTION fc) const { // TODO : tester
        auto forceOnMarker2 = GetLinkReactionForceOnMarker2(fc);
        return m_node2->GetFrameWRT_COG_InBody().ProjectVectorFrameInParent(forceOnMarker2, fc);
    }

    const Torque FrLink_::GetLinkReactionTorqueOnMarker1(FRAME_CONVENTION fc) const { // TODO : tester
        auto torque = m_chronoLink->c_generalizedForceOnMarker1.GetTorque();
        if (IsNED(fc)) internal::SwapFrameConvention<Torque>(torque);
        return torque;
    }

    const Torque FrLink_::GetLinkReactionTorqueOnMarker2(FRAME_CONVENTION fc) const { // TODO : tester
        auto torque = m_chronoLink->c_generalizedForceOnMarker2.GetTorque();
        if (IsNED(fc)) internal::SwapFrameConvention<Torque>(torque);
        return torque;
    }

    const Torque FrLink_::GetLinkReactionTorqueOnBody1AtCOG(FRAME_CONVENTION fc) const { // TODO : tester
        auto markerFrame_WRT_COG = m_node1->GetFrameWRT_COG_InBody();

        auto torqueAtMarker1_ref = markerFrame_WRT_COG.ProjectVectorFrameInParent<Torque>(GetLinkReactionForceOnMarker1(fc), fc);
        auto COG_M1_ref = markerFrame_WRT_COG.GetPosition(fc);
        auto force_ref = markerFrame_WRT_COG.ProjectVectorFrameInParent<Force>(GetLinkReactionForceOnMarker1(fc), fc);

        return torqueAtMarker1_ref + COG_M1_ref.cross(force_ref);
    }

    const Torque FrLink_::GetLinkReactionTorqueOnBody2AtCOG(FRAME_CONVENTION fc) const { // TODO : tester
        auto markerFrame_WRT_COG = m_node2->GetFrameWRT_COG_InBody();

        auto torqueAtMarker2_ref = markerFrame_WRT_COG.ProjectVectorFrameInParent<Torque>(GetLinkReactionForceOnMarker2(fc), fc);
        auto COG_M2_ref = markerFrame_WRT_COG.GetPosition(fc);
        auto force_ref = markerFrame_WRT_COG.ProjectVectorFrameInParent<Force>(GetLinkReactionForceOnMarker2(fc), fc);

        return torqueAtMarker2_ref + COG_M2_ref.cross(force_ref);
    }

    void FrLink_::Initialize() {
        SetMarkers(m_node1.get(), m_node2.get());
    }

    void FrLink_::Update(double time) {
        // TODO : Ici, on met en cache les differentes quantites utiles a FrLink_ mise en convention frydom ie
        // les donnes de la liaison sont relatives a un mouvement du node 2 par rapport au node 1
        // On fait appel aux methodes de FrLinkLockBase pour faciliter
        // Du coup, l'update de l'objet Chrono doit etre fait avant l'objet frydom


    }

    void FrLink_::SetLinkForceAtMarker1(const Force &force) {
        // force doit etre exprime dans le repere 1
        // On doit exprimer la force dans le repere 2
        m_chronoLink->GetLinkForce() =
                - internal::Vector3dToChVector(GetMarker1FrameWRTMarker2Frame().ProjectVectorFrameInParent<Force>(force, NWU));
    }

    void FrLink_::SetLinkTorqueAtMarker1(const Torque &torque) {

    }

    const Force FrLink_::GetLinkForceAtMarker1(FRAME_CONVENTION fc) const {

    }

    const Torque FrLink_::GetLinkTorqueAtMarker1(FRAME_CONVENTION fc) const {

    }

    double FrLink_::GetLinkPower() const {
        return GetLinkForceAtMarker1(NWU).dot(GetVelocityOfMarker2WRTMarker1(NWU))
             + GetLinkTorqueAtMarker1(NWU).dot(GetAngularVelocityOfMarker2WRTMarker1(NWU));
    }


}  // end namespace frydom
