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

#ifndef FRYDOM_FRDOFMASKLINK_H
#define FRYDOM_FRDOFMASKLINK_H

#include "FrLink.h"

namespace frydom {

    /*
* Defining a mask class to make the constraint on bodies WRT to world easier
*/

    /**
     * \class FrDOFMask
     * \brief Class for defining the constraints on bodies with respect to world easier.
     */
    class FrDOFMask {

     private:

      LINK_TYPE m_linkType = FREE_LINK;
      bool m_xLocked = false;
      bool m_yLocked = false;
      bool m_zLocked = false;
      bool m_RxLocked = false;
      bool m_RyLocked = false;
      bool m_RzLocked = false;

     public:

      // TODO : plutot utiliser ChLinkLockMaskLF en interne et faire des conversions pour les angles vers les coeffs de quaternion
      /*
       * Pour les angles, un blocage en
       */

      /// If true, locks the X DOF of the body
      void SetLock_X(bool lock);

      /// If true, locks the Y DOF of the body
      void SetLock_Y(bool lock);

      /// If true, locks the Z DOF of the body
      void SetLock_Z(bool lock);

      /// If true, locks the RX DOF of the body
      void SetLock_Rx(bool lock);

      /// If true, locks the RY DOF of the body
      void SetLock_Ry(bool lock);

      /// If true, locks the RZ DOF of the body
      void SetLock_Rz(bool lock);

      /// Locking the body in the world vertical plane
      void LockXZPlane();  // On bloque y, rx, rz

      /// Locking the body in the world horizontal plane
      void LockXYPlane();  // On bloque z, ry

      /// Is the X DOF locked ?
      bool GetLock_X() const;

      /// Is the Y DOF locked ?
      bool GetLock_Y() const;

      /// Is the Z DOF locked ?
      bool GetLock_Z() const;

      /// Is the RX DOF locked ?
      bool GetLock_Rx() const;

      /// Is the RY DOF locked ?
      bool GetLock_Ry() const;

      /// Is the RZ DOF locked ?
      bool GetLock_Rz() const;

      /// Returns true if the body has some locked DOF
      bool HasLockedDOF() const;

      /// Returns true if the body has no embedded constraint
      bool IsFree() const;

      /// Removes every constraint from the body WRT to the world
      void MakeItFree();

      /// Makes the body fixed in the world
      void MakeItLocked();

      /// Returns the number of locked DOF of the body WRT the world
      unsigned int GetNbLockedDOF() const;

      /// Returns the number of constrained DOF of the body WRT the world
      unsigned int GetNbFreeDOF() const;

      void SetLinkType(LINK_TYPE linkType);

      LINK_TYPE GetLinkType() const;

     private:

      void SetLock(bool xLocked, bool yLocked, bool zLocked, bool rxLocked, bool ryLocked, bool rzLocked);

    };


    /**
     * \class FrDOFMaskLink
     * \brief Class to deal with constraints between bodies and the world, based on FrDOFMask. Derived from FrLink.
     */
    class FrDOFMaskLink : public FrLink {

     public:

      /// Constructor taking the nodes attached to the two bodies implied in the link and the system
      FrDOFMaskLink(const std::string &name,
                    const std::shared_ptr<FrNode> &node1,
                    const std::shared_ptr<FrNode> &node2,
                    FrOffshoreSystem *system);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "DOFMaskLink"; }

      /// Set the FrDOFMask. Essentially used by the DOF restricting mechanism of bodies
      /// Users should not use this method to make links between bodies but directly use the specialized classes
      /// (FrPrismaticLink, FrRevoluteLink...)
      void SetDOFMask(FrDOFMask *mask);

    };

} // end namespace frydom

#endif //FRYDOM_FRDOFMASKLINK_H
