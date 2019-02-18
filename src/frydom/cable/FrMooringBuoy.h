//
// Created by Lucas Letournel on 21/08/18.
//

#ifndef FRYDOM_FRBUOY_H
#define FRYDOM_FRBUOY_H

#include "frydom/core/FrBodyEasy.h"
#include "frydom/hydrodynamics/FrLinearDamping.h"


namespace frydom {

    class FrMooringBuoy : public FrSphere {
    private:

        class FrSphereNonLinearHydrostaticForce : public FrForce{
        private:
        public:
            FrSphereNonLinearHydrostaticForce() {
                force.SetNull();
                moment.SetNull();
            }

            void UpdateState() override {
                auto m_buoy = dynamic_cast<FrMooringBuoy*>(GetBody());
                auto Gvector = m_buoy->GetSystem()->Get_G_acc();
                auto rho_water = dynamic_cast<FrOffshoreSystem*>(m_buoy->GetSystem())->GetEnvironment()->GetWaterDensity() ;
                force = -m_buoy->GetVolume()*rho_water*Gvector;
            };
        };

        double m_radius;
        double m_volume;
        std::shared_ptr<FrSphereNonLinearHydrostaticForce> m_hydrostaticForce;
        std::shared_ptr<FrLinearDamping> m_dampingForce;

    public:
        FrMooringBuoy(double radius, double mass, bool visual_asset = true, double damping=0)
                :FrSphere(radius, mass, visual_asset){
            m_radius = radius;
            m_hydrostaticForce = std::make_shared<FrSphereNonLinearHydrostaticForce>();
            m_hydrostaticForce->SetBody(this);
            AddForce(m_hydrostaticForce);
            m_dampingForce = std::make_shared<FrLinearDamping>();
            m_dampingForce->SetDiagonalDamping(damping,damping,damping,damping,damping,damping);
            AddForce(m_dampingForce);
        };

        double computeDraft(){
            if (GetPos().z()<-m_radius) { return m_radius;}
            if (GetPos().z()>m_radius) {return -m_radius;}
            else {return -GetPos().z();}
        }

        double inline computeVolume(){
            auto Zt = computeDraft();
            m_volume = M_PI/3 * (Zt*(3*m_radius*m_radius - Zt*Zt) + 2*pow(m_radius,3));
        }

        double GetVolume() {return m_volume;}


        void Update(bool update_assets = true) override{

            computeVolume();

            // update parent class
            chrono::ChBodyAuxRef::Update(update_assets);
        }


    };

}
#endif //FRYDOM_FRBUOY_H
