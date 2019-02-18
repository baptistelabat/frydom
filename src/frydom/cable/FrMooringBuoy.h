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

    class FrLinearDamping_;

    class FrMooringBuoy : public FrBody_ {
    private:

        class FrSphereNonLinearHydrostaticForce : public FrForce_{
        public:
            void Update(double time) override;
            void StepFinalize() override {};
        };


        double m_radius = 1.;
        double c_volume;
        std::shared_ptr<FrSphereNonLinearHydrostaticForce> m_hydrostaticForce;
        std::shared_ptr<FrLinearDamping_> m_dampingForce;

    public:

        FrMooringBuoy(double radius, double mass, bool visual_asset = true, double damping=0);;

        double GetVolume() {return c_volume;}


        void Update() override;

    private:

        double computeDraft();

        double inline computeVolume(){
            auto Zt = computeDraft();
            c_volume = M_PI/3 * (Zt*(3*m_radius*m_radius - Zt*Zt) + 2*std::pow(m_radius,3));
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
    make_mooring_buoy(FrOffshoreSystem_* system, double radius, double mass, bool visual_asset = true, double damping=0);
} //end namespace frydom

#endif //FRYDOM_FRMOORINGBUOY_H
