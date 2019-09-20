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

#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/physics/ChLinkMasked.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"
#include "actuators/FrActuator.h"
#include "FrDOFMaskLink.h"

#include "frydom/core/link/FrLinkMaskBase.h"
#include "frydom/core/link/constraint/FrConstraintTwoBodiesBase.h"


namespace frydom {

    namespace internal {

        template<typename OffshoreSystemType>
        FrLinkLockBase<OffshoreSystemType>::FrLinkLockBase(FrLink<OffshoreSystemType> *frydomLink) : m_frydomLink(
            frydomLink), chrono::ChLinkLock() {}

        template<typename OffshoreSystemType>
        void FrLinkLockBase<OffshoreSystemType>::SetLinkType(LINK_TYPE lt) {
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
            case SPHERICAL:
              ChangeLinkType(ChronoLinkType::SPHERICAL);
              break;
            case PRISMATICREVOLUTE:
              ChangeLinkType(ChronoLinkType::REVOLUTEPRISMATIC);
              break;
            case CUSTOM:

              break;
//                    case SCREW:
//                        ChangeLinkType(ChronoLinkType::CYLINDRICAL);
//                        break;
//                case PERPENDICULAR:
//                    ChangeLinkType(ChronoLinkType::PERPEND);
//                    break;
//                case PARALLEL:
//                    ChangeLinkType(ChronoLinkType::PARALLEL);
//                    break;
//                case PLANEONPLANE:
//                    ChangeLinkType(ChronoLinkType::PLANEPLANE);
//                    break;
//                case DISTANCETOAXIS:
//                    ChangeLinkType(ChronoLinkType::);
//                    break;
//                case POINTONLINE:
//                    ChangeLinkType(ChronoLinkType::POINTLINE);
//                    break;
//                case POINTONPLANE:
//                    ChangeLinkType(ChronoLinkType::POINTPLANE);
//                    break;
//                case POINTONSPLINE:
//                    ChangeLinkType(ChronoLinkType::);
//                    break;
          }
        }

        template<typename OffshoreSystemType>
        void FrLinkLockBase<OffshoreSystemType>::SetupInitial() {

        }

        template<typename OffshoreSystemType>
        void FrLinkLockBase<OffshoreSystemType>::Update(double time, bool update_assets) {

          chrono::ChLinkLock::Update(time, update_assets);

          GenerateCache();

          m_frydomLink->Update(time);

        }

        template<typename OffshoreSystemType>
        void FrLinkLockBase<OffshoreSystemType>::GenerateCache() {
          // 1 - Relative Frames
          c_frame1WRT2 = internal::ChCoordsys2FrFrame(GetRelM());
          c_frame2WRT1 = c_frame1WRT2.GetInverse();

          // 2 - Relative velocities
          c_generalizedVelocity1WRT2.SetVelocity(internal::ChVectorToVector3d<Velocity>(GetRelM_dt().pos));
          c_generalizedVelocity1WRT2.SetAngularVelocity(internal::ChVectorToVector3d<AngularVelocity>(GetRelWvel()));

          c_generalizedVelocity2WRT1.SetVelocity(
              -c_frame2WRT1.ProjectVectorFrameInParent<Velocity>(c_generalizedVelocity1WRT2.GetVelocity(), NWU));
          c_generalizedVelocity2WRT1.SetAngularVelocity(
              -c_frame2WRT1.ProjectVectorFrameInParent<AngularVelocity>(c_generalizedVelocity1WRT2.GetAngularVelocity(),
                                                                        NWU));

          // 2 - Relative accelerations
          c_generalizedAcceleration1WRT2.SetAcceleration(
              internal::ChVectorToVector3d<Acceleration>(GetRelM_dtdt().pos));
          c_generalizedAcceleration1WRT2.SetAngularAcceleration(
              internal::ChVectorToVector3d<AngularAcceleration>(GetRelWacc()));

          c_generalizedAcceleration2WRT1.SetAcceleration(
              -c_frame2WRT1.ProjectVectorFrameInParent<Acceleration>(c_generalizedAcceleration1WRT2.GetAcceleration(),
                                                                     NWU));
          c_generalizedAcceleration2WRT1.SetAngularAcceleration(
              -c_frame2WRT1.ProjectVectorFrameInParent<AngularAcceleration>(
                  c_generalizedAcceleration1WRT2.GetAngularAcceleration(), NWU));

          // 3 - Link forces
          c_generalizedForceOnNode2.SetForce(internal::ChVectorToVector3d<Force>(Get_react_force()));
          c_generalizedForceOnNode2.SetTorque(internal::ChVectorToVector3d<Torque>(Get_react_torque()));

          c_generalizedForceOnNode1.SetForce(
              -c_frame2WRT1.ProjectVectorFrameInParent<Force>(c_generalizedForceOnNode2.GetForce(), NWU));
          c_generalizedForceOnNode1.SetTorque(
              -c_frame2WRT1.ProjectVectorFrameInParent(c_generalizedForceOnNode2.GetTorque(), NWU)
              + c_frame2WRT1.GetPosition(NWU).cross(c_generalizedForceOnNode1.GetForce())
          );

        }

        template<typename OffshoreSystemType>
        void FrLinkLockBase<OffshoreSystemType>::SetMask(FrDOFMask *vmask) {

          if (vmask->GetLinkType() == LINK_TYPE::CUSTOM) {
            FrLinkMaskBase<OffshoreSystemType> chronoMask;
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

        template<typename OffshoreSystemType>
        void FrLinkLockBase<OffshoreSystemType>::SetLinkForceOnBody1InFrame2AtOrigin1(const Force &force) {
          C_force = internal::Vector3dToChVector(force);
        }

        template<typename OffshoreSystemType>
        void FrLinkLockBase<OffshoreSystemType>::SetLinkTorqueOnBody1InFrame2AtOrigin1(const Torque &torque) {
          C_torque = internal::Vector3dToChVector(torque);
        }

        template<typename OffshoreSystemType>
        Force FrLinkLockBase<OffshoreSystemType>::GetLinkForceOnBody1InFrame2AtOrigin1() {
          return internal::ChVectorToVector3d<Force>(C_force);
        }

        template<typename OffshoreSystemType>
        Torque FrLinkLockBase<OffshoreSystemType>::GetLinkTorqueOnBody1InFrame2ArOrigin1() {
          return internal::ChVectorToVector3d<Torque>(C_torque);
        }

        template<typename OffshoreSystemType>
        FrFrame
        FrLinkLockBase<OffshoreSystemType>::GetConstraintViolation() { // TODO : voir si c'est bien la violation de 2 par rapport a 1, dans 1 !! sinon, renvoyer l'inverse
          return internal::ChCoordsys2FrFrame(GetRelC());
        }

        template<typename OffshoreSystemType>
        void FrLinkLockBase<OffshoreSystemType>::BuildLinkType(chrono::ChLinkLock::LinkType link_type) {

          type = link_type;

          FrLinkMaskBase<OffshoreSystemType> m_mask;

          // SetLockMask() sets the constraints for the link coordinates: (X,Y,Z, E0,E1,E2,E3)
          switch (type) {
            case LinkType::FREE:
              m_mask.SetLockMask(false, false, false, false, false, false, false);
              break;
            case LinkType::LOCK:
              m_mask.SetLockMask(true, true, true, false, true, true, true);
              break;
            case LinkType::SPHERICAL:
              m_mask.SetLockMask(true, true, true, false, false, false, false);
              break;
            case LinkType::POINTPLANE:
              m_mask.SetLockMask(false, false, true, false, false, false, false);
              break;
            case LinkType::POINTLINE:
              m_mask.SetLockMask(false, true, true, false, false, false, false);
              break;
            case LinkType::REVOLUTE:
              m_mask.SetLockMask(true, true, true, false, true, true, false);
              break;
            case LinkType::CYLINDRICAL:
              m_mask.SetLockMask(true, true, false, false, true, true, false);
              break;
            case LinkType::PRISMATIC:
              m_mask.SetLockMask(true, true, false, false, true, true, true);
              break;
            case LinkType::PLANEPLANE:
              m_mask.SetLockMask(false, false, true, false, true, true, false);
              break;
            case LinkType::OLDHAM:
              m_mask.SetLockMask(false, false, true, false, true, true, true);
              break;
            case LinkType::ALIGN:
              m_mask.SetLockMask(false, false, false, false, true, true, true);
            case LinkType::PARALLEL:
              m_mask.SetLockMask(false, false, false, false, true, true, false);
              break;
            case LinkType::PERPEND:
              m_mask.SetLockMask(false, false, false, false, true, false, true);
              break;
            case LinkType::REVOLUTEPRISMATIC:
              m_mask.SetLockMask(false, true, true, false, true, true, false);
              break;
            default:
              m_mask.SetLockMask(false, false, false, false, false, false, false);
              break;
          }

          BuildLink(&m_mask);
        }


    }  // end namespace frydom::internal


    /*
     * FrLink method definitions
     *
     */
    template<typename OffshoreSystemType>
    FrLink<OffshoreSystemType>::FrLink(const std::shared_ptr<FrNode<OffshoreSystemType>> &node1,
                                       const std::shared_ptr<FrNode<OffshoreSystemType>> &node2,
                                       FrOffshoreSystem<OffshoreSystemType> *system) :
        FrLinkBase<OffshoreSystemType>(node1, node2, system),
        m_frame2WRT1_reference() {
      m_chronoLink = std::make_shared<internal::FrLinkLockBase>(this);
      this->SetLogged(true);
      m_actuator = nullptr;
    }

    template<typename OffshoreSystemType>
    void FrLink<OffshoreSystemType>::SetNodes(FrNode<OffshoreSystemType> *node1, FrNode<OffshoreSystemType> *node2) {
      // IMPORTANT : in FRyDoM the first node is the master and the second one the slave, as opposed to Chrono !!!
      m_chronoLink->ReferenceMarkers(node2->m_chronoMarker.get(), node1->m_chronoMarker.get());
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<chrono::ChLink> FrLink<OffshoreSystemType>::GetChronoLink() {
      return m_chronoLink;
    }

    template<typename OffshoreSystemType>
    internal::FrLinkLockBase<OffshoreSystemType> *FrLink<OffshoreSystemType>::GetChronoItem_ptr() const {
      return m_chronoLink.get();
    }

    template<typename OffshoreSystemType>
    bool FrLink<OffshoreSystemType>::IsDisabled() const {
      return m_chronoLink->IsDisabled();
    }

    template<typename OffshoreSystemType>
    void FrLink<OffshoreSystemType>::SetDisabled(bool disabled) {
      m_chronoLink->SetDisabled(disabled);
      if (IsMotorized()) {
        m_actuator->SetDisabled(disabled);
      }
    }

    template<typename OffshoreSystemType>
    void FrLink<OffshoreSystemType>::SetBreakable(bool breakable) {
      m_breakable = breakable;
    }

    template<typename OffshoreSystemType>
    bool FrLink<OffshoreSystemType>::IsBreakable() const {
      return m_breakable;
    }

    template<typename OffshoreSystemType>
    bool FrLink<OffshoreSystemType>::IsBroken() const {
      return m_chronoLink->IsBroken();
    }

    template<typename OffshoreSystemType>
    void FrLink<OffshoreSystemType>::SetBroken(bool broken) {
      if (!IsBreakable()) return;

      m_chronoLink->SetBroken(broken);
      if (IsMotorized()) {
        m_actuator->SetDisabled(broken);
      }
    }

    template<typename OffshoreSystemType>
    bool FrLink<OffshoreSystemType>::IsActive() const {
      return m_chronoLink->IsActive();
    }

    template<typename OffshoreSystemType>
    bool FrLink<OffshoreSystemType>::IsMotorized() const {
      return (m_actuator && m_actuator->IsActive());
    }

    template<typename OffshoreSystemType>
    void FrLink<OffshoreSystemType>::SetThisConfigurationAsReference() {
      m_frame2WRT1_reference = m_chronoLink->c_frame2WRT1;  // FIXME : cette methode devrait trigger l'update des caches de classes derivees
      UpdateCache();
    }

    // Must be reimplemented in
//    bool FrLink<OffshoreSystemType>::IsMotorized() const { return false; }
    template<typename OffshoreSystemType>
    const FrFrame FrLink<OffshoreSystemType>::GetNode2FrameWRTNode1Frame() const {
      return m_chronoLink->c_frame2WRT1;
    }

    template<typename OffshoreSystemType>
    const FrFrame FrLink<OffshoreSystemType>::GetNode1FrameWRTNode2Frame() const {
      return m_chronoLink->c_frame1WRT2;
    }

    template<typename OffshoreSystemType>
    const Position FrLink<OffshoreSystemType>::GetNode2PositionWRTNode1(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_frame2WRT1.GetPosition(fc);
    }

    template<typename OffshoreSystemType>
    const Position FrLink<OffshoreSystemType>::GetNode1PositionWRTNode2(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_frame1WRT2.GetPosition(fc);
    }

    template<typename OffshoreSystemType>
    const FrRotation FrLink<OffshoreSystemType>::GetNode2OrientationWRTNode1() const {
      return m_chronoLink->c_frame2WRT1.GetRotation();
    }

    template<typename OffshoreSystemType>
    const FrRotation FrLink<OffshoreSystemType>::GetNode1OrientationWRTNode2() const {
      return m_chronoLink->c_frame1WRT2.GetRotation();
    }

    template<typename OffshoreSystemType>
    const GeneralizedVelocity
    FrLink<OffshoreSystemType>::GetGeneralizedVelocityOfNode2WRTNode1(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_generalizedVelocity2WRT1;
    }

    template<typename OffshoreSystemType>
    const GeneralizedVelocity
    FrLink<OffshoreSystemType>::GetGeneralizedVelocityOfNode1WRTNode2(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_generalizedVelocity1WRT2;
    }

    template<typename OffshoreSystemType>
    const Velocity FrLink<OffshoreSystemType>::GetVelocityOfNode2WRTNode1(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_generalizedVelocity2WRT1.GetVelocity();
    }

    template<typename OffshoreSystemType>
    const Velocity FrLink<OffshoreSystemType>::GetVelocityOfNode1WRTNode2(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_generalizedVelocity1WRT2.GetVelocity();
    }

    template<typename OffshoreSystemType>
    const AngularVelocity FrLink<OffshoreSystemType>::GetAngularVelocityOfNode2WRTNode1(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_generalizedVelocity2WRT1.GetAngularVelocity();
    }

    template<typename OffshoreSystemType>
    const AngularVelocity FrLink<OffshoreSystemType>::GetAngularVelocityOfNode1WRTNode2(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_generalizedVelocity1WRT2.GetAngularVelocity();
    }

    template<typename OffshoreSystemType>
    const GeneralizedAcceleration
    FrLink<OffshoreSystemType>::GetGeneralizedAccelerationOfNode2WRTNode1(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_generalizedAcceleration2WRT1;
    }

    template<typename OffshoreSystemType>
    const GeneralizedAcceleration
    FrLink<OffshoreSystemType>::GetGeneralizedAccelerationOfNode1WRTNode2(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_generalizedAcceleration1WRT2;
    }

    template<typename OffshoreSystemType>
    const Acceleration FrLink<OffshoreSystemType>::GetAccelerationOfNode1WRTNode2(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_generalizedAcceleration1WRT2.GetAcceleration();
    }

    template<typename OffshoreSystemType>
    const Acceleration FrLink<OffshoreSystemType>::GetAccelerationOfNode2WRTNode1(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_generalizedAcceleration2WRT1.GetAcceleration();
    }

    template<typename OffshoreSystemType>
    const AngularAcceleration
    FrLink<OffshoreSystemType>::GetAngularAccelerationOfNode2WRTNode1(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_generalizedAcceleration2WRT1.GetAngularAcceleration();
    }

    template<typename OffshoreSystemType>
    const AngularAcceleration
    FrLink<OffshoreSystemType>::GetAngularAccelerationOfNode1WRTNode2(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_generalizedAcceleration1WRT2.GetAngularAcceleration();
    }

    template<typename OffshoreSystemType>
    const Force FrLink<OffshoreSystemType>::GetLinkReactionForceOnNode1(FRAME_CONVENTION fc) const { // TODO : tester
      auto force = m_chronoLink->c_generalizedForceOnNode1.GetForce();
      if (IsNED(fc)) internal::SwapFrameConvention<Force>(force);
      return force;
    }

    template<typename OffshoreSystemType>
    const Force FrLink<OffshoreSystemType>::GetLinkReactionForceOnNode2(FRAME_CONVENTION fc) const { // TODO : tester
      auto force = m_chronoLink->c_generalizedForceOnNode2.GetForce();
      if (IsNED(fc)) internal::SwapFrameConvention<Force>(force);
      return force;
    }

    template<typename OffshoreSystemType>
    const Force FrLink<OffshoreSystemType>::GetLinkReactionForceOnBody1(FRAME_CONVENTION fc) const { // TODO : tester
      auto forceOnNode1 = GetLinkReactionForceOnNode1(fc);
      return this->m_node1->GetFrameWRT_COG_InBody().ProjectVectorFrameInParent(forceOnNode1, fc);
    }

    template<typename OffshoreSystemType>
    const Force FrLink<OffshoreSystemType>::GetLinkReactionForceOnBody2(FRAME_CONVENTION fc) const { // TODO : tester
      auto forceOnNode2 = GetLinkReactionForceOnNode2(fc);
      return this->m_node2->GetFrameWRT_COG_InBody().ProjectVectorFrameInParent(forceOnNode2, fc);
    }

    template<typename OffshoreSystemType>
    const Torque FrLink<OffshoreSystemType>::GetLinkReactionTorqueOnNode1(FRAME_CONVENTION fc) const { // TODO : tester
      auto torque = m_chronoLink->c_generalizedForceOnNode1.GetTorque();
      if (IsNED(fc)) internal::SwapFrameConvention<Torque>(torque);
      return torque;
    }

    template<typename OffshoreSystemType>
    const Torque FrLink<OffshoreSystemType>::GetLinkReactionTorqueOnNode2(FRAME_CONVENTION fc) const { // TODO : tester
      auto torque = m_chronoLink->c_generalizedForceOnNode2.GetTorque();
      if (IsNED(fc)) internal::SwapFrameConvention<Torque>(torque);
      return torque;
    }

    template<typename OffshoreSystemType>
    const Torque
    FrLink<OffshoreSystemType>::GetLinkReactionTorqueOnBody1AtCOG(FRAME_CONVENTION fc) const { // TODO : tester
      auto nodeFrame_WRT_COG = this->m_node1->GetFrameWRT_COG_InBody();

      auto torqueAtNode1_ref = nodeFrame_WRT_COG.template ProjectVectorFrameInParent<Torque>(
          GetLinkReactionTorqueOnNode1(fc),
          fc);
      auto COG_M1_ref = nodeFrame_WRT_COG.GetPosition(fc);
      auto force_ref = nodeFrame_WRT_COG.template ProjectVectorFrameInParent<Force>(GetLinkReactionForceOnNode1(fc),
                                                                                    fc);

      return torqueAtNode1_ref + COG_M1_ref.cross(force_ref);
    }

    template<typename OffshoreSystemType>
    const Torque
    FrLink<OffshoreSystemType>::GetLinkReactionTorqueOnBody2AtCOG(FRAME_CONVENTION fc) const { // TODO : tester
      auto nodeFrame_WRT_COG = this->m_node2->GetFrameWRT_COG_InBody();

      auto torqueAtNode2_ref = nodeFrame_WRT_COG.template ProjectVectorFrameInParent<Torque>(
          GetLinkReactionTorqueOnNode2(fc),
          fc);
      auto COG_M2_ref = nodeFrame_WRT_COG.GetPosition(fc);
      auto force_ref = nodeFrame_WRT_COG.template ProjectVectorFrameInParent<Force>(GetLinkReactionForceOnNode2(fc),
                                                                                    fc);

      return torqueAtNode2_ref + COG_M2_ref.cross(force_ref);
    }

    template<typename OffshoreSystemType>
    void FrLink<OffshoreSystemType>::Initialize() {

      SetNodes(this->m_node1.get(), this->m_node2.get());
      m_chronoLink->SetupInitial();
    }

    template<typename OffshoreSystemType>
    void FrLink<OffshoreSystemType>::Update(double time) {
      // TODO : Ici, on met en cache les differentes quantites utiles a FrLink mise en convention frydom ie
      // les donnes de la liaison sont relatives a un mouvement du node 2 par rapport au node 1
      // On fait appel aux methodes de FrLinkLockBase pour faciliter
      // Du coup, l'update de l'objet Chrono doit etre fait avant l'objet frydom

    }

    template<typename OffshoreSystemType>
    void
    FrLink<OffshoreSystemType>::SetLinkForceTorqueOnBody2InFrame2AtOrigin2(const Force &force, const Torque &torque) {
      /* From Chrono comments in ChLinkMasked::UpdateForces :
       * C_force and C_torque   are considered in the reference coordsystem
       * of marker2  (the MAIN marker), and their application point is considered the
       * origin of marker1 (the SLAVE marker)
       */

      // Force
      m_chronoLink->SetLinkForceOnBody1InFrame2AtOrigin1(
          -force); // The minus signe is to get the force applied on body 1

      // Torque transport
      auto O1O2 = -m_chronoLink->c_frame1WRT2.GetPosition(NWU);

      Torque torque_M1 = -torque + O1O2.cross(-force);
      m_chronoLink->SetLinkTorqueOnBody1InFrame2AtOrigin1(torque_M1);
    }

    template<typename OffshoreSystemType>
    const Force FrLink<OffshoreSystemType>::GetLinkForceOnBody1InFrame1AtOrigin1(FRAME_CONVENTION fc) const {
      auto force = m_chronoLink->GetLinkForceOnBody1InFrame2AtOrigin1();
      return m_chronoLink->c_frame2WRT1.template ProjectVectorFrameInParent<Force>(force, fc);
    }

    template<typename OffshoreSystemType>
    const Force FrLink<OffshoreSystemType>::GetLinkForceOnBody2InFrame2AtOrigin2(FRAME_CONVENTION fc) const {
      return -m_chronoLink->GetLinkForceOnBody1InFrame2AtOrigin1();
    }

    template<typename OffshoreSystemType>
    const Torque FrLink<OffshoreSystemType>::GetLinkTorqueOnBody1InFrame1AtOrigin1(FRAME_CONVENTION fc) const {
      auto torque = m_chronoLink->GetLinkTorqueOnBody1InFrame2ArOrigin1();
      return m_chronoLink->c_frame2WRT1.template ProjectVectorFrameInParent<Force>(torque, fc);
    }

    template<typename OffshoreSystemType>
    const Torque FrLink<OffshoreSystemType>::GetLinkTorqueOnBody2InFrame2AtOrigin2(FRAME_CONVENTION fc) const {
      auto torque_O1_2 = m_chronoLink->GetLinkTorqueOnBody1InFrame2ArOrigin1();
      auto force_2 = m_chronoLink->GetLinkForceOnBody1InFrame2AtOrigin1();
      auto O2O1_2 = m_chronoLink->c_frame1WRT2.GetPosition(NWU);
      return -(torque_O1_2 + O2O1_2.cross(force_2));
    }

    template<typename OffshoreSystemType>
    const Force FrLink<OffshoreSystemType>::GetLinkForceOnBody1InFrame2AtOrigin1(FRAME_CONVENTION fc) const {
      return m_chronoLink->GetLinkForceOnBody1InFrame2AtOrigin1();
    }

    template<typename OffshoreSystemType>
    const Force FrLink<OffshoreSystemType>::GetLinkForceOnBody2InFrame1AtOrigin2(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_frame2WRT1.template ProjectVectorFrameInParent<Force>(
          GetLinkForceOnBody2InFrame2AtOrigin2(fc), fc);
    }

    template<typename OffshoreSystemType>
    const Torque FrLink<OffshoreSystemType>::GetLinkTorqueOnBody1InFrame2AtOrigin1(FRAME_CONVENTION fc) const {
      return m_chronoLink->GetLinkTorqueOnBody1InFrame2ArOrigin1();
    }

    template<typename OffshoreSystemType>
    const Torque FrLink<OffshoreSystemType>::GetLinkTorqueOnBody2InFrame1AtOrigin2(FRAME_CONVENTION fc) const {
      return m_chronoLink->c_frame2WRT1.template ProjectVectorFrameInParent<Torque>(
          GetLinkTorqueOnBody2InFrame2AtOrigin2(fc),
          fc);
    }

    template<typename OffshoreSystemType>
    double FrLink<OffshoreSystemType>::GetLinkPower() const {
      return GetLinkForceOnBody2InFrame1AtOrigin2(NWU).dot(GetVelocityOfNode2WRTNode1(NWU))
             + GetLinkTorqueOnBody2InFrame1AtOrigin2(NWU).dot(GetAngularVelocityOfNode2WRTNode1(NWU));
    }

    template<typename OffshoreSystemType>
    void FrLink<OffshoreSystemType>::AddFields() {
      this->m_message->template AddField<double>("time", "s", "Current time of the simulation",
                                                 [this]() { return this->m_system->GetTime(); });

      // Node Position
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("PositionOfNode2WRTNode1", "m",
           fmt::format("Node 2 position relatively to Node 1, in Node 1 reference frame in {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetNode2PositionWRTNode1(this->GetLogFrameConvention()); });
      // Node Velocity
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("VelocityOfNode2WRTNode1", "m/s",
           fmt::format("Node 2 velocity relatively to Node 1, in Node 1 reference frame in {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetVelocityOfNode2WRTNode1(this->GetLogFrameConvention()); });
      // Node Acceleration
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("AccelerationOfNode2WRTNode1", "m/s^2",
           fmt::format("Node 2 acceleration relatively to Node 1, in Marker 1 reference frame in {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetAccelerationOfNode2WRTNode1(this->GetLogFrameConvention()); });

      // Node Orientation
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("OrientationOfNode2WRTNode1", "rad",
           fmt::format("Node 2 orientation relatively to Node 1, in Node 1 reference frame in {}",
                       this->GetLogFrameConvention()),
           [this]() {
             double phi, theta, psi;
             GetNode2OrientationWRTNode1().GetCardanAngles_RADIANS(phi, theta, psi, this->GetLogFrameConvention());
             return Position(phi, theta, psi);
           });
      // Node Angular Velocity
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("AngularVelocityOfNode2WRTNode1", "rad/s",
           fmt::format("Node 2 angular velocity relatively to Node 1, in Node 1 reference frame in {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetAngularVelocityOfNode2WRTNode1(this->GetLogFrameConvention()); });
      // Node Angular Acceleration
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("AngularAccelerationOfNode2WRTNode1", "m/s^2",
           fmt::format("Node 2 angular acceleration relatively to Node 1, in Node 1 reference frame in {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetAngularAccelerationOfNode2WRTNode1(this->GetLogFrameConvention()); });


      // Force
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("LinkReactionForceOnBody1", "N",
           fmt::format("link reaction force applied at marker 1, expressed in body 1 reference frame in {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetLinkReactionForceOnBody1(this->GetLogFrameConvention()); });
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("LinkReactionForceOnBody2", "N",
           fmt::format("link reaction force applied at marker 2, expressed in body 2 reference frame in {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetLinkReactionForceOnBody2(this->GetLogFrameConvention()); });
      // Torque
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("LinkReactionTorqueOnBody1", "Nm",
           fmt::format("link reaction torque at Node 1, expressed in Node 1 reference frame in {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetLinkReactionTorqueOnNode1(this->GetLogFrameConvention()); });
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("LinkReactionTorqueOnBody2", "Nm",
           fmt::format("link reaction torque at Node 2, expressed in Node 2 reference frame in {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetLinkReactionTorqueOnNode2(this->GetLogFrameConvention()); });

      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("LinkReactionTorqueOnBody1AtCOG", "Nm",
           fmt::format("link reaction torque at CoG applied at marker 1, expressed in body 1 reference frame in {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetLinkReactionTorqueOnBody1AtCOG(this->GetLogFrameConvention()); });
      this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
          ("LinkReactionTorqueOnBody2AtCOG", "Nm",
           fmt::format("link reaction torque at CoG applied at marker 2, expressed in body 2 reference frame in {}",
                       this->GetLogFrameConvention()),
           [this]() { return GetLinkReactionTorqueOnBody2AtCOG(this->GetLogFrameConvention()); });

      // Power
      this->m_message->template AddField<double>
          ("LinkPower", "kW", "power delivered in a FrLink", [this]() { return 0.001 * GetLinkPower(); });

      // Motor
      if (m_actuator) {
        this->m_message->template AddField<double>
            ("MotorPower", "kW", "power delivered by the motor", [this]() { return m_actuator->GetMotorPower(); });
        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("MotorForceInBody1", "N",
             fmt::format("Force applied by the motor on body 1, in body 1 reference frame {}",
                         this->GetLogFrameConvention()),
             [this]() { return m_actuator->GetMotorForceInBody1(this->GetLogFrameConvention()); });
        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("MotorForceInBody2", "N",
             fmt::format("Force applied by the motor on body 1, in body 2 reference frame {}",
                         this->GetLogFrameConvention()),
             [this]() { return m_actuator->GetMotorForceInBody2(this->GetLogFrameConvention()); });
        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("MotorTorqueInBody1(", "Nm",
             fmt::format("Torque applied by the motor on body 1, in body 1 reference frame {}",
                         this->GetLogFrameConvention()),
             [this]() { return m_actuator->GetMotorTorqueInBody1(this->GetLogFrameConvention()); });
        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("MotorTorqueInBody2(", "Nm",
             fmt::format("Torque applied by the motor on body 2, in body 2 reference frame {}",
                         this->GetLogFrameConvention()),
             [this]() { return m_actuator->GetMotorTorqueInBody2(this->GetLogFrameConvention()); });
        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("MotorTorqueAtCOGInBody1(", "Nm",
             fmt::format("Torque applied by the motor at COG on body 1, in body 1 reference frame {}",
                         this->GetLogFrameConvention()),
             [this]() { return m_actuator->GetMotorTorqueAtCOGInBody1(this->GetLogFrameConvention()); });
        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("MotorTorqueAtCOGInBody2(", "Nm",
             fmt::format("Torque applied by the motor at COG on body 2, in body 2 reference frame {}",
                         this->GetLogFrameConvention()),
             [this]() { return m_actuator->GetMotorTorqueAtCOGInBody2(this->GetLogFrameConvention()); });

      }

    }

    template<typename OffshoreSystemType>
    FrFrame FrLink<OffshoreSystemType>::GetConstraintViolation() const {
      return m_chronoLink->GetConstraintViolation();
    }

    template<typename OffshoreSystemType>
    void FrLink<OffshoreSystemType>::UpdateCache() {}

}  // end namespace frydom
