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

#include "FrDOFMaskLink.h"

namespace frydom {


    /*
     * FrDOFMask definitions
     */

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

      switch (m_linkType) {
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
          SetLock(true, true, true, false, false, false);
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
      m_linkType = LINK_TYPE::CUSTOM;
    }


    /*
     * FrDOFMaskLink definitions
     */


    void FrDOFMaskLink::SetDOFMask(FrDOFMask *mask) {
      m_chronoLink->SetMask(mask);
    }

    FrDOFMaskLink::FrDOFMaskLink(const std::string &&name,
                                 const std::shared_ptr<FrNode> &node1,
                                 const std::shared_ptr<FrNode> &node2,
                                 FrOffshoreSystem *system) :
        FrLink(std::move(name), node1, node2, system) {}


}
