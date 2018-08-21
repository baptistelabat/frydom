//
// Created by Lucas Letournel on 21/08/18.
//

#ifndef FRYDOM_FRBUOY_H
#define FRYDOM_FRBUOY_H

#include "frydom/core/FrBodyEasy.h"


namespace frydom {

    class FrMooringBuoy : public FrSphere {
    private:

        class FrSphereNonLinearHydrostaticForce : public FrForce{
        private:
            FrMooringBuoy* m_buoy;
        public:
            //void UpdateTime(double mytime) override{};
            void UpdateState() override {
                auto Gvector = m_buoy->GetSystem()->Get_G_acc();
                auto rho_water = dynamic_cast<FrOffshoreSystem*>(m_buoy->GetSystem())->GetEnvironment()->GetWaterDensity() ;
                force = m_buoy->GetMass()*Gvector + m_buoy->GetVolume()*rho_water;
            };
        };

        double m_radius;
        double m_volume;
        std::shared_ptr<FrSphereNonLinearHydrostaticForce> m_force;

    public:
        FrMooringBuoy(double radius, double mass, bool visual_asset = true)
                :FrSphere(radius, mass, visual_asset){
            m_radius = radius;
            m_force = std::make_shared<FrSphereNonLinearHydrostaticForce>();
            m_force->SetBody(this);
            AddForce(m_force);
        };

        double computeDraft(){
            if (GetPos().z()<m_radius) { return m_radius;}
            if (GetPos().z()>m_radius) {return 0;}
            else {return m_radius - GetPos().z();}
        }

        double inline computeVolume(double Zt){
            m_volume = M_PI/3 * (Zt*(3*m_radius*m_radius - Zt*Zt) + 2*pow(m_radius,3));
        }

        double GetVolume() {return m_volume;}


        void Update(bool update_assets = true) override{

            m_volume = computeVolume(computeDraft());

            // update parent class
            chrono::ChBodyAuxRef::Update(update_assets);

        }


    };

}
#endif //FRYDOM_FRBUOY_H
