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

#include "frydom/logging/FrLogManager.h"


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

    void FrLinkLockBase::SetupInitial() {

    }

    void FrLinkLockBase::Update(double time, bool update_assets) {

      chrono::ChLinkLock::Update(time, update_assets);

      GenerateCache();

      m_frydomLink->Update(time);

    }

    void FrLinkLockBase::GenerateCache() {
      // 1 - Relative Frames
            c_frame2WRT1 = internal::ChCoordsys2FrFrame(GetRelM());
            c_frame1WRT2 = c_frame2WRT1.GetInverse();

      // 2 - Relative velocities
            c_generalizedVelocity2WRT1.SetVelocity(internal::ChVectorToVector3d<Velocity>(GetRelM_dt().pos));
            c_generalizedVelocity2WRT1.SetAngularVelocity(internal::ChVectorToVector3d<AngularVelocity>(GetRelWvel()));

            c_generalizedVelocity1WRT2.SetVelocity(
                    - c_frame1WRT2.ProjectVectorFrameInParent<Velocity>(c_generalizedVelocity2WRT1.GetVelocity(), NWU));
            c_generalizedVelocity1WRT2.SetAngularVelocity(
                    - c_frame1WRT2.ProjectVectorFrameInParent<AngularVelocity>(c_generalizedVelocity2WRT1.GetAngularVelocity(), NWU));

      // 2 - Relative accelerations
            c_generalizedAcceleration2WRT1.SetAcceleration(internal::ChVectorToVector3d<Acceleration>(GetRelM_dtdt().pos));
            c_generalizedAcceleration2WRT1.SetAngularAcceleration(internal::ChVectorToVector3d<AngularAcceleration>(GetRelWacc()));

            c_generalizedAcceleration1WRT2.SetAcceleration(
                    - c_frame1WRT2.ProjectVectorFrameInParent<Acceleration>(c_generalizedAcceleration2WRT1.GetAcceleration(), NWU));
            c_generalizedAcceleration1WRT2.SetAngularAcceleration(
                    - c_frame1WRT2.ProjectVectorFrameInParent<AngularAcceleration>(c_generalizedAcceleration2WRT1.GetAngularAcceleration(), NWU));

      // 3 - Link forces
            c_generalizedForceOnNode1.SetForce(internal::ChVectorToVector3d<Force>(Get_react_force()));
            c_generalizedForceOnNode1.SetTorque(internal::ChVectorToVector3d<Torque>(Get_react_torque()));

            c_generalizedForceOnNode2.SetForce(
                    - c_frame1WRT2.ProjectVectorFrameInParent<Force>(c_generalizedForceOnNode1.GetForce(), NWU));
            c_generalizedForceOnNode2.SetTorque(
                    - c_frame1WRT2.ProjectVectorFrameInParent(c_generalizedForceOnNode1.GetTorque(), NWU)
                    + c_frame1WRT2.GetPosition(NWU).cross(c_generalizedForceOnNode2.GetForce())
      );

    }

    void FrLinkLockBase::SetMask(FrDOFMask *vmask) {

      if (vmask->GetLinkType() == LINK_TYPE::CUSTOM) {
        FrLinkMaskBase chronoMask;
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

    void FrLinkLockBase::SetLinkTorqueOnBody1InFrame2AtOrigin1(const Torque &torque) {
      C_torque = internal::Vector3dToChVector(torque);
    }

    Force FrLinkLockBase::GetLinkForceOnBody1InFrame2AtOrigin1() {
      return internal::ChVectorToVector3d<Force>(C_force);
    }

        Torque FrLinkLockBase::GetLinkTorqueOnBody1InFrame2AtOrigin1() {
      return internal::ChVectorToVector3d<Torque>(C_torque);
    }

      Force FrLinkLockBase::GetLinkForceOnNode1() const {
        return -GetLinkForceOnNode2();
      }

      Torque FrLinkLockBase::GetLinkTorqueOnNode1() const {
        return -GetLinkTorqueOnNode2();
      }

      Force FrLinkLockBase::GetLinkForceOnNode2() const {
        return internal::ChVectorToVector3d<Force>(C_force);
      }

      Torque FrLinkLockBase::GetLinkTorqueOnNode2() const {
            return internal::ChVectorToVector3d<Torque>(C_torque);
        }

        FrFrame FrLinkLockBase::GetConstraintViolation() { // TODO : voir si c'est bien la violation de 2 par rapport a 1, dans 1 !! sinon, renvoyer l'inverse
      return internal::ChCoordsys2FrFrame(GetRelC());
    }

    void FrLinkLockBase::BuildLinkType(chrono::ChLinkLock::LinkType link_type) {

      type = link_type;

      FrLinkMaskBase m_mask;

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

  FrLink::FrLink(const std::string &name,
                 const std::string &type_name,
                 FrOffshoreSystem *system,
                 const std::shared_ptr<FrNode> &node1,
                 const std::shared_ptr<FrNode> &node2) :
      FrLinkBase(name, type_name, system, node1, node2),
      m_frame2WRT1_reference() {

    m_chronoLink = std::make_shared<internal::FrLinkLockBase>(this);
//        SetLogged(true);
    m_actuator = nullptr;
  }


  void FrLink::SetNodes(FrNode *node1, FrNode *node2) {
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

  void FrLink::DefineLogMessages() {

    auto msg = NewMessage("State", "State message");

    msg->AddField<double>("time", "s", "Current time of the simulation",
                          [this]() { return GetSystem()->GetTime(); });

    // Node Position
    msg->AddField<Eigen::Matrix<double, 3, 1>>
            ("PositionOfNode2WRTNode1","m", fmt::format("Node 2 position relatively to Node 1, in Node 1 reference frame in {}",
                                                        GetLogFC()),
             [this]() {return GetNode2PositionWRTNode1(GetLogFC());});
    // Node Velocity
    msg->AddField<Eigen::Matrix<double, 3, 1>>
            ("VelocityOfNode2WRTNode1","m/s", fmt::format("Node 2 velocity relatively to Node 1, in Node 1 reference frame in {}", GetLogFC()),
             [this]() {return GetVelocityOfNode2WRTNode1(GetLogFC());});
    // Node Acceleration
    msg->AddField<Eigen::Matrix<double, 3, 1>>
            ("AccelerationOfNode2WRTNode1","m/s^2", fmt::format("Node 2 acceleration relatively to Node 1, in Marker 1 reference frame in {}", GetLogFC()),
             [this]() {return GetAccelerationOfNode2WRTNode1(GetLogFC());});

    // Node Orientation
    msg->AddField<Eigen::Matrix<double, 3, 1>>
            ("OrientationOfNode2WRTNode1","rad", fmt::format("Node 2 orientation relatively to Node 1, in Node 1 reference frame in {}", GetLogFC()),
             [this]() {double phi, theta, psi; GetNode2OrientationWRTNode1().GetCardanAngles_RADIANS(phi, theta, psi, GetLogFC());
                return Position(phi, theta, psi);});
    // Node Angular Velocity
    msg->AddField<Eigen::Matrix<double, 3, 1>>
            ("AngularVelocityOfNode2WRTNode1","rad/s", fmt::format("Node 2 angular velocity relatively to Node 1, in Node 1 reference frame in {}", GetLogFC()),
             [this]() {return GetAngularVelocityOfNode2WRTNode1(GetLogFC());});
    // Node Angular Acceleration
    msg->AddField<Eigen::Matrix<double, 3, 1>>
            ("AngularAccelerationOfNode2WRTNode1","m/s^2", fmt::format("Node 2 angular acceleration relatively to Node 1, in Node 1 reference frame in {}", GetLogFC()),
             [this]() {return GetAngularAccelerationOfNode2WRTNode1(GetLogFC());});
    // Force
    msg->AddField<Eigen::Matrix<double, 3, 1>>
            ("LinkReactionForceOnBody1","N", fmt::format("link reaction force applied at marker 1, expressed in body 1 reference frame in {}", GetLogFC()),
             [this]() {return GetLinkReactionForceOnBody1(GetLogFC());});
    msg->AddField<Eigen::Matrix<double, 3, 1>>
            ("LinkReactionForceOnBody2","N", fmt::format("link reaction force applied at marker 2, expressed in body 2 reference frame in {}", GetLogFC()),
             [this]() {return GetLinkReactionForceOnBody2(GetLogFC());});
    // Torque
    msg->AddField<Eigen::Matrix<double, 3, 1>>
            ("LinkReactionTorqueOnBody1","Nm", fmt::format("link reaction torque at Node 1, expressed in Node 1 reference frame in {}", GetLogFC()),
             [this]() {return GetLinkReactionTorqueOnNode1(GetLogFC());});
    msg->AddField<Eigen::Matrix<double, 3, 1>>
            ("LinkReactionTorqueOnBody2","Nm", fmt::format("link reaction torque at Node 2, expressed in Node 2 reference frame in {}", GetLogFC()),
             [this]() {return GetLinkReactionTorqueOnNode2(GetLogFC());});

    msg->AddField<Eigen::Matrix<double, 3, 1>>
            ("LinkReactionTorqueOnBody1AtCOG","Nm", fmt::format("link reaction torque at CoG applied at marker 1, expressed in body 1 reference frame in {}", GetLogFC()),
             [this]() {return GetLinkReactionTorqueOnBody1AtCOG(GetLogFC());});
    msg->AddField<Eigen::Matrix<double, 3, 1>>
            ("LinkReactionTorqueOnBody2AtCOG","Nm", fmt::format("link reaction torque at CoG applied at marker 2, expressed in body 2 reference frame in {}", GetLogFC()),
             [this]() {return GetLinkReactionTorqueOnBody2AtCOG(GetLogFC());});

    // Power
    msg->AddField<double>
            ("LinkPower","kW", "power delivered in a FrLink", [this]() {return 0.001*GetLinkPower();});

    // Motor
    if (m_actuator) {
      msg->AddField<double>
          ("MotorPower", "kW", "power delivered by the motor", [this]() { return m_actuator->GetMotorPower(); });
      msg->AddField<Eigen::Matrix<double, 3, 1>>
          ("MotorForceInBody1", "N",
           fmt::format("Force applied by the motor on body 1, in body 1 reference frame {}", GetLogFC()),
           [this]() { return m_actuator->GetMotorForceInBody1(GetLogFC()); });
      msg->AddField<Eigen::Matrix<double, 3, 1>>
          ("MotorForceInBody2", "N",
           fmt::format("Force applied by the motor on body 1, in body 2 reference frame {}", GetLogFC()),
           [this]() { return m_actuator->GetMotorForceInBody2(GetLogFC()); });
      msg->AddField<Eigen::Matrix<double, 3, 1>>
          ("MotorTorqueInBody1(", "Nm",
           fmt::format("Torque applied by the motor on body 1, in body 1 reference frame {}", GetLogFC()),
           [this]() { return m_actuator->GetMotorTorqueInBody1(GetLogFC()); });
      msg->AddField<Eigen::Matrix<double, 3, 1>>
          ("MotorTorqueInBody2(", "Nm",
           fmt::format("Torque applied by the motor on body 2, in body 2 reference frame {}", GetLogFC()),
           [this]() { return m_actuator->GetMotorTorqueInBody2(GetLogFC()); });
      msg->AddField<Eigen::Matrix<double, 3, 1>>
          ("MotorTorqueAtCOGInBody1(", "Nm",
           fmt::format("Torque applied by the motor at COG on body 1, in body 1 reference frame {}", GetLogFC()),
           [this]() { return m_actuator->GetMotorTorqueAtCOGInBody1(GetLogFC()); });
      msg->AddField<Eigen::Matrix<double, 3, 1>>
          ("MotorTorqueAtCOGInBody2(", "Nm",
           fmt::format("Torque applied by the motor at COG on body 2, in body 2 reference frame {}", GetLogFC()),
           [this]() { return m_actuator->GetMotorTorqueAtCOGInBody2(GetLogFC()); });
    }


  }

  const FrFrame FrLink::GetNode2FrameWRTNode1Frame() const {
    return m_chronoLink->c_frame2WRT1;
  }

  const FrFrame FrLink::GetNode1FrameWRTNode2Frame() const {
    return m_chronoLink->c_frame1WRT2;
  }

  const Position FrLink::GetNode2PositionWRTNode1(FRAME_CONVENTION fc) const {
    return m_chronoLink->c_frame2WRT1.GetPosition(fc);
  }

  const Position FrLink::GetNode1PositionWRTNode2(FRAME_CONVENTION fc) const {
    return m_chronoLink->c_frame1WRT2.GetPosition(fc);
  }

  const FrRotation FrLink::GetNode2OrientationWRTNode1() const {
    return m_chronoLink->c_frame2WRT1.GetRotation();
  }

  const FrRotation FrLink::GetNode1OrientationWRTNode2() const {
    return m_chronoLink->c_frame1WRT2.GetRotation();
  }

  const GeneralizedVelocity FrLink::GetGeneralizedVelocityOfNode2WRTNode1(FRAME_CONVENTION fc) const {
    return m_chronoLink->c_generalizedVelocity2WRT1;
  }

  const GeneralizedVelocity FrLink::GetGeneralizedVelocityOfNode1WRTNode2(FRAME_CONVENTION fc) const {
    return m_chronoLink->c_generalizedVelocity1WRT2;
  }

  const Velocity FrLink::GetVelocityOfNode2WRTNode1(FRAME_CONVENTION fc) const {
    return m_chronoLink->c_generalizedVelocity2WRT1.GetVelocity();
  }

  const Velocity FrLink::GetVelocityOfNode1WRTNode2(FRAME_CONVENTION fc) const {
    return m_chronoLink->c_generalizedVelocity1WRT2.GetVelocity();
  }

  const AngularVelocity FrLink::GetAngularVelocityOfNode2WRTNode1(FRAME_CONVENTION fc) const {
    return m_chronoLink->c_generalizedVelocity2WRT1.GetAngularVelocity();
  }

  const AngularVelocity FrLink::GetAngularVelocityOfNode1WRTNode2(FRAME_CONVENTION fc) const {
    return m_chronoLink->c_generalizedVelocity1WRT2.GetAngularVelocity();
  }

  const GeneralizedAcceleration FrLink::GetGeneralizedAccelerationOfNode2WRTNode1(FRAME_CONVENTION fc) const {
    return m_chronoLink->c_generalizedAcceleration2WRT1;
  }

  const GeneralizedAcceleration FrLink::GetGeneralizedAccelerationOfNode1WRTNode2(FRAME_CONVENTION fc) const {
    return m_chronoLink->c_generalizedAcceleration1WRT2;
  }

  const Acceleration FrLink::GetAccelerationOfNode1WRTNode2(FRAME_CONVENTION fc) const {
    return m_chronoLink->c_generalizedAcceleration1WRT2.GetAcceleration();
  }

  const Acceleration FrLink::GetAccelerationOfNode2WRTNode1(FRAME_CONVENTION fc) const {
    return m_chronoLink->c_generalizedAcceleration2WRT1.GetAcceleration();
  }

  const AngularAcceleration FrLink::GetAngularAccelerationOfNode2WRTNode1(FRAME_CONVENTION fc) const {
    return m_chronoLink->c_generalizedAcceleration2WRT1.GetAngularAcceleration();
  }

  const AngularAcceleration FrLink::GetAngularAccelerationOfNode1WRTNode2(FRAME_CONVENTION fc) const {
    return m_chronoLink->c_generalizedAcceleration1WRT2.GetAngularAcceleration();
  }

    const Force FrLink::GetLinkReactionForceOnNode1(FRAME_CONVENTION fc) const {
    auto force = m_chronoLink->c_generalizedForceOnNode1.GetForce();
    if (IsNED(fc)) internal::SwapFrameConvention<Force>(force);
    return force;
  }

    const Force FrLink::GetLinkReactionForceOnNode2(FRAME_CONVENTION fc) const {
    auto force = m_chronoLink->c_generalizedForceOnNode2.GetForce();
    if (IsNED(fc)) internal::SwapFrameConvention<Force>(force);
    return force;
  }

    const Force FrLink::GetLinkReactionForceOnBody1(FRAME_CONVENTION fc) const {
    auto forceOnNode1 = GetLinkReactionForceOnNode1(fc);
    return m_node1->GetFrameWRT_COG_InBody().ProjectVectorFrameInParent(forceOnNode1, fc);
  }

    const Force FrLink::GetLinkReactionForceOnBody2(FRAME_CONVENTION fc) const {
    auto forceOnNode2 = GetLinkReactionForceOnNode2(fc);
    return m_node2->GetFrameWRT_COG_InBody().ProjectVectorFrameInParent(forceOnNode2, fc);
  }

    const Torque FrLink::GetLinkReactionTorqueOnNode1(FRAME_CONVENTION fc) const {
    auto torque = m_chronoLink->c_generalizedForceOnNode1.GetTorque();
    if (IsNED(fc)) internal::SwapFrameConvention<Torque>(torque);
    return torque;
  }

    const Torque FrLink::GetLinkReactionTorqueOnNode2(FRAME_CONVENTION fc) const {
    auto torque = m_chronoLink->c_generalizedForceOnNode2.GetTorque();
    if (IsNED(fc)) internal::SwapFrameConvention<Torque>(torque);
    return torque;
  }

    const Torque FrLink::GetLinkReactionTorqueOnBody1AtCOG(FRAME_CONVENTION fc) const {
    auto nodeFrame_WRT_COG = m_node1->GetFrameWRT_COG_InBody();

    auto torqueAtNode1_ref =
        nodeFrame_WRT_COG.ProjectVectorFrameInParent<Torque>(GetLinkReactionTorqueOnNode1(fc), fc);
    auto COG_M1_ref = nodeFrame_WRT_COG.GetPosition(fc);
    auto force_ref = nodeFrame_WRT_COG.ProjectVectorFrameInParent<Force>(GetLinkReactionForceOnNode1(fc), fc);

    return torqueAtNode1_ref + COG_M1_ref.cross(force_ref);
  }

    const Torque FrLink::GetLinkReactionTorqueOnBody2AtCOG(FRAME_CONVENTION fc) const {
    auto nodeFrame_WRT_COG = m_node2->GetFrameWRT_COG_InBody();

    auto torqueAtNode2_ref =
        nodeFrame_WRT_COG.ProjectVectorFrameInParent<Torque>(GetLinkReactionTorqueOnNode2(fc), fc);
    auto COG_M2_ref = nodeFrame_WRT_COG.GetPosition(fc);
    auto force_ref = nodeFrame_WRT_COG.ProjectVectorFrameInParent<Force>(GetLinkReactionForceOnNode2(fc), fc);

    return torqueAtNode2_ref + COG_M2_ref.cross(force_ref);
  }

  void FrLink::Initialize() {

    SetNodes(m_node1.get(), m_node2.get());
    m_chronoLink->SetupInitial();

//    GetSystem()->GetLogManager()->Add(this);
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
    m_chronoLink->SetLinkForceOnBody1InFrame2AtOrigin1(
        -force); // The minus signe is to get the force applied on body 1

    // Torque transport
    auto O1O2 = -m_chronoLink->c_frame1WRT2.GetPosition(NWU);

    Torque torque_M1 = -torque + O1O2.cross(-force);
    m_chronoLink->SetLinkTorqueOnBody1InFrame2AtOrigin1(torque_M1);
  }


  double FrLink::GetLinkPower() const {
        return GetSpringDamperForceOnNode2(NWU).dot(GetVelocityOfNode2WRTNode1(NWU))
             + GetSpringDamperTorqueOnNode2(NWU).dot(GetAngularVelocityOfNode2WRTNode1(NWU));
  }

  FrFrame FrLink::GetConstraintViolation() const {
    return m_chronoLink->GetConstraintViolation();
  }

  void FrLink::UpdateCache() {}
  const Force FrLink::GetSpringDamperForceOnNode1(FRAME_CONVENTION fc) const {
    auto force = m_chronoLink->GetLinkForceOnNode1();
    if (IsNED(fc)) {internal::SwapFrameConvention(force);}
    return force;
  }

  const Torque FrLink::GetSpringDamperTorqueOnNode1(FRAME_CONVENTION fc) const {
    auto torque = m_chronoLink->GetLinkTorqueOnNode1();
    if (IsNED(fc)) {internal::SwapFrameConvention(torque);}
    return torque;
  }

  const Force FrLink::GetSpringDamperForceOnNode2(FRAME_CONVENTION fc) const {
    auto force = m_chronoLink->GetLinkForceOnNode2();
    if (IsNED(fc)) {internal::SwapFrameConvention(force);}
    return force;
  }

  const Torque FrLink::GetSpringDamperTorqueOnNode2(FRAME_CONVENTION fc) const {
    auto torque = m_chronoLink->GetLinkTorqueOnNode2();
    if (IsNED(fc)) {internal::SwapFrameConvention(torque);}
    return torque;
  }

  const Force FrLink::GetSpringDamperForceOnBody1(FRAME_CONVENTION fc) const {
    auto force = GetSpringDamperForceOnNode1(fc);
    return m_node1->GetFrameWRT_COG_InBody().ProjectVectorFrameInParent(force, fc);
  }

  const Force FrLink::GetSpringDamperForceOnBody2(FRAME_CONVENTION fc) const {
      auto force = GetSpringDamperForceOnNode2(fc);
      return m_node2->GetFrameWRT_COG_InBody().ProjectVectorFrameInParent(force, fc);

  }

  const Torque FrLink::GetSpringDamperTorqueOnBody1AtCOG(FRAME_CONVENTION fc) const {
      auto nodeFrame_WRT_COG = m_node1->GetFrameWRT_COG_InBody();

      auto torqueAtNode1_ref = nodeFrame_WRT_COG.ProjectVectorFrameInParent<Torque>(GetSpringDamperTorqueOnNode1(fc), fc);
      auto COG_M1_ref = nodeFrame_WRT_COG.GetPosition(fc);
      auto force_ref = nodeFrame_WRT_COG.ProjectVectorFrameInParent<Force>(GetSpringDamperForceOnNode1(fc), fc);

      return torqueAtNode1_ref + COG_M1_ref.cross(force_ref);
    }

  const Torque FrLink::GetSpringDamperTorqueOnBody2AtCOG(FRAME_CONVENTION fc) const {
      auto nodeFrame_WRT_COG = m_node2->GetFrameWRT_COG_InBody();

      auto torqueAtNode2_ref = nodeFrame_WRT_COG.ProjectVectorFrameInParent<Torque>(GetSpringDamperTorqueOnNode2(fc), fc);
      auto COG_M2_ref = nodeFrame_WRT_COG.GetPosition(fc);
      auto force_ref = nodeFrame_WRT_COG.ProjectVectorFrameInParent<Force>(GetSpringDamperForceOnNode2(fc), fc);

      return torqueAtNode2_ref + COG_M2_ref.cross(force_ref);
    }

//  const GeneralizedForceTorsor FrLink::GetLinkReactionForceOnNode1_(FRAME_CONVENTION fc) const {
//      return GeneralizedForceTorsor(frydom::Force(), frydom::Torque(), frydom::Position(), NWU);
//  }


}  // end namespace frydom
