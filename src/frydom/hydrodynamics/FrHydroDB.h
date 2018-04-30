//
// Created by frongere on 17/10/17.
//

#ifndef FRYDOM_FRHYDRODB_H
#define FRYDOM_FRHYDRODB_H

#include <vector>
#include "MathUtils/MathUtils.h"
#include "frydom/utils/FrEigen.h" // TODO: Eigen est maintenant deja importe de MathUtils... ne plus reposer sur le sous module frydom

#include "FrBEMBody.h"

// TODO: utiliser plutot des std::vector a la place des matrices eigen ...

using namespace mathutils;

namespace frydom {

    // Forward declarations
    class FrHydroMapper;


    class FrHydroDB {

    // =================================================================================================================

    private:

        double m_GravityAcc            = 9.81;
        double m_NormalizationLength   = 1.;
        double m_WaterDensity          = 1000.;
        double m_WaterDepth            = 0.;

        std::vector<std::shared_ptr<FrBEMBody>> m_Bodies;
        FrDiscretization1D m_FrequencyDiscretization;
        FrDiscretization1D m_WaveDirectionDiscretization;
        FrDiscretization1D m_TimeDiscretization;

    public:

        FrHydroDB() = default;

        unsigned int GetNbBodies() const { return (unsigned int)m_Bodies.size(); }

        double GetGravityAcc() const { return m_GravityAcc; }

        void SetGravityAcc(const double GravityAcc) { m_GravityAcc = GravityAcc; }

        double GetNormalizationLength() const { return m_NormalizationLength; }

        void SetNormalizationLength(const double NormalizationLength) { m_NormalizationLength = NormalizationLength; }

        double GetWaterDensity() const { return m_WaterDensity; }

        void SetWaterDensity(const double WaterDensity) { m_WaterDensity = WaterDensity; }

        double GetWaterDepth() const { return m_WaterDepth; }

        void SetWaterDepth(const double WaterDepth) { m_WaterDepth = WaterDepth; }

        void SetFrequencyDiscretization(const double MinFreq, const double MaxFreq, const unsigned int NbFreq) {
            m_FrequencyDiscretization.SetMin(MinFreq);
            m_FrequencyDiscretization.SetMax(MaxFreq);
            m_FrequencyDiscretization.SetNbSample(NbFreq);
        }

        std::vector<double> GetFrequencies() const { return m_FrequencyDiscretization.GetVector(); } // TODO: gerer les unites...

        void SetWaveDirectionDiscretization(const double MinAngle, const double MaxAngle, const unsigned int NbAngle) {
            m_WaveDirectionDiscretization.SetMin(MinAngle);
            m_WaveDirectionDiscretization.SetMax(MaxAngle);
            m_WaveDirectionDiscretization.SetNbSample(NbAngle);
        }

        std::vector<double> GetWaveDirections() const { return m_WaveDirectionDiscretization.GetVector(); } // TODO: gerer les unites...

        void SetTimeDiscretization(const double FinalTime, const unsigned int NbTimeSamples) {
            m_TimeDiscretization.SetMin(0.);
            m_TimeDiscretization.SetMax(FinalTime);
            m_TimeDiscretization.SetNbSample(NbTimeSamples);
        }

        unsigned int GetNbFrequencies() const { return m_FrequencyDiscretization.GetNbSample(); }

        double GetMaxFrequency() const { return m_FrequencyDiscretization.GetMax(); }

        double GetMinFrequency() const { return m_FrequencyDiscretization.GetMin(); }

        double GetStepFrequency() const {return m_FrequencyDiscretization.GetStep(); }

        unsigned int GetNbWaveDirections() const { return m_WaveDirectionDiscretization.GetNbSample(); }

        unsigned int GetNbTimeSamples() const { return m_TimeDiscretization.GetNbSample(); }

        double GetFinalTime() const { return m_TimeDiscretization.GetMax(); }

        double GetTimeStep() const { return m_TimeDiscretization.GetStep(); }

        std::shared_ptr<FrBEMBody> NewBody(std::string BodyName) {
            auto BEMBody = std::make_shared<FrBEMBody>(GetNbBodies(), BodyName);
            BEMBody->SetHDB(this);
            m_Bodies.push_back(BEMBody);
            return BEMBody;
        }

        std::shared_ptr<FrBEMBody> GetBody(unsigned int ibody) { return m_Bodies[ibody]; }

        void GenerateImpulseResponseFunctions(double tf = 30., double dt = 0.);

        std::shared_ptr<FrHydroMapper> GetMapper();

    };  // end class FrHydroDB

}  // end namespace frydom


#endif //FRYDOM_FRHYDRODB_H
