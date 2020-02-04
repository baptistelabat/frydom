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

#ifndef FRYDOM_FRMOORINGBUOY_H
#define FRYDOM_FRMOORINGBUOY_H

#include "frydom/core/body/FrBody.h"
#include "frydom/core/force/FrForce.h"

namespace frydom {

  // Forward declaration
  class FrLinearDamping;


  class FrMooringBuoy : public FrBody {

   private:

    class FrSphereNonLinearHydrostaticForce : public FrForce {

     public:

      explicit FrSphereNonLinearHydrostaticForce(const std::string &name, FrBody *body) :
          FrForce(name, "NonlinearHydrostaticsOnSphericBuoy", body) {} // TODO : Refactorer completement cette classe !!

      /// Return true if the force is included in the static analysis
      bool IncludedInStaticAnalysis() const override { return true; }

     private:

      /// Compute the nonlinear hydrostatic force for a sphere
      /// \param time Current time of the simulation from beginning, in seconds
      void Compute(double time) override;
    };


    double m_radius = 1.;
    double c_volume;
    std::shared_ptr<FrSphereNonLinearHydrostaticForce> m_hydrostaticForce;
    std::shared_ptr<FrLinearDamping> m_dampingForce;

   public:

    FrMooringBuoy(const std::string &name,
                  FrOffshoreSystem *system,
                  double radius,
                  double mass,
                  bool visual_asset = true,
                  double damping = 0);

    double GetVolume() { return c_volume; }


    void Update() override;

   private:

    double computeDraft();
    
    double inline computeVolume() {
      auto Zt = computeDraft();
      return c_volume = M_PI / 3 * (Zt * (3 * m_radius * m_radius - Zt * Zt) + 2 * std::pow(m_radius, 3));
    }

  };

  /// Maker for the mooring buoy class: instantiate and return a FrMooringBuoy. It also add it to the system provided.
  /// \param system system in charge of the buoy
  /// \param radius radius of the buoy
  /// \param mass mass of the buoy
  /// \param visual_asset true if an asset is to be viewed
  /// \param damping damping coefficient affected to the diagonal terms of a linear damping force.
  /// \return FrMooringBuoy instance
  std::shared_ptr<FrMooringBuoy>
  make_mooring_buoy(const std::string &name, FrOffshoreSystem *system, double radius, double mass,
                    bool visual_asset = true, double damping = 0);

}  //end namespace frydom

#endif //FRYDOM_FRMOORINGBUOY_H
