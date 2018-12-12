//
// Created by Lucas Letournel on 22/11/18.
//

#include "FrOcean_.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/flow/FrFlowBase.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

#include "freeSurface/FrFreeSurface.h"
#include "seabed/FrSeabed.h"

#include "frydom/asset/FrSeabedGridAsset.h"
#include "frydom/asset/FrFreeSurfaceGridAsset.h"


namespace frydom{

    FrOcean_::FrOcean_(FrEnvironment_* environment) :m_environment(environment) {

        m_seabed        = std::make_unique<FrSeabed_>(this);
        m_freeSurface   = std::make_unique<FrFreeSurface_>(this);
        m_current       = std::make_unique<FrCurrent_>(this);
        m_waterProp     = std::make_unique<FrFluidProperties>(10., 1027., 0.001397, 1.3604E-06, 35., 1.2030E-03 );

    }

    FrEnvironment_ *FrOcean_::GetEnvironment() const { return m_environment;}


    void FrOcean_::SetTemperature(double Temperature) {m_waterProp->m_temperature = Temperature;}

    double FrOcean_::GetTemperature() const {return m_waterProp->m_temperature;}

    void FrOcean_::SetDensity(double Density) {m_waterProp->m_density = Density;}

    double FrOcean_::GetDensity() const {return m_waterProp->m_density;}

    void FrOcean_::SetDynamicViscosity(double DynamicViscosity) {m_waterProp->m_dynamicViscosity = DynamicViscosity;}

    double FrOcean_::GetDynamicViscosity() const {return m_waterProp->m_dynamicViscosity;}

    void FrOcean_::SetKinematicViscosity(double KinematicViscosity) {m_waterProp->m_kinematicViscosity = KinematicViscosity;}

    double FrOcean_::GetKinematicViscosity() const {return m_waterProp->m_kinematicViscosity;}

    void FrOcean_::SetSalinity(double Salinity) {m_waterProp->m_salinity = Salinity;}

    double FrOcean_::GetSalinity() const {return m_waterProp->m_salinity;}

    void FrOcean_::SetPressure(double Pressure) {m_waterProp->m_pressure = Pressure;}

    double FrOcean_::GetPressure() const {return m_waterProp->m_pressure;}

    double FrOcean_::GetReynoldsNumberInWater(double characteristicLength, double velocity) const {
        return fabs(velocity) * characteristicLength / GetKinematicViscosity();
    }

    double FrOcean_::GetFroudeNumberInWater(double characteristicLength, double velocity) const {
        return fabs(velocity) / sqrt(m_environment->GetGravityAcceleration() * characteristicLength);
    }

    FrFreeSurface_ *FrOcean_::GetFreeSurface() const { return m_freeSurface.get();}

    FrCurrent_ *FrOcean_::GetCurrent() const { return m_current.get();}

    FrSeabed_ *FrOcean_::GetSeabed() const { return m_seabed.get();}

    void FrOcean_::Update(double time) {

        m_freeSurface->Update(time);
        m_current->Update(time);
        m_seabed->Update(time);

    }

    void FrOcean_::Initialize() {
        m_freeSurface->Initialize();
        m_current->Initialize();
        m_seabed->Initialize();
    }

    void FrOcean_::StepFinalize() {
        m_freeSurface->StepFinalize();
        m_current->StepFinalize();
        m_seabed->StepFinalize();
    }

    double FrOcean_::GetDepth(FRAME_CONVENTION fc) const {
        return m_freeSurface->GetTidal()->GetHeight(fc) - m_seabed->GetBathymetry(fc);
    }

    double FrOcean_::GetDepth(double x, double y, FRAME_CONVENTION fc) const {
        return m_freeSurface->GetTidal()->GetHeight(fc) - m_seabed->GetBathymetry(x,y,fc);
    }

    void FrOcean_::ShowSeabed(bool showSeabed) {
        m_seabed->ShowSeabed(showSeabed);
    }

    void FrOcean_::ShowFreeSurface(bool showFreeSurface) {
        m_freeSurface->ShowFreeSurface(showFreeSurface);
    }

}