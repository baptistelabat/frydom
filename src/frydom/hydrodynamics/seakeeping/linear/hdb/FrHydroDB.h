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

        FrHydroDB(std::string h5file);

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

        void GenerateSpeedDependentIRF();

        std::shared_ptr<FrHydroMapper> GetMapper();

        void IntLoadResidual_Mv(const unsigned int off, chrono::ChVectorDynamic<>& Res,
                                const chrono::ChVectorDynamic<>& w, const double c) ;

        //void VariablesFbIncrementMq();

    };  // end class FrHydroDB














    /// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    // Forward declarations

    class FrHydroMapper_;
    class FrBEMBody_;
    class FrBody_;
    class FrHDF5Reader;
    class FrEquilibriumFrame_;

    // ----------------------------------------------------------
    // FrDiscretization1D
    // ----------------------------------------------------------

    class FrDiscretization1D_ {
    private:
        double m_xmin = 0.;
        double m_xmax = 0.;
        unsigned int m_nx = 0;

    public:

        FrDiscretization1D_() = default;

        FrDiscretization1D_(double xmin, double xmax, unsigned int nx)
                : m_xmin(xmin), m_xmax(xmax), m_nx(nx) {}

        double GetMin() const { return m_xmin; }
        void SetMin(double xmin) { m_xmin = xmin; }

        double GetMax() const { return m_xmax; }
        void SetMax(double xmax) { m_xmax = xmax; }

        unsigned int GetNbSample() const { return m_nx; }
        void SetNbSample(unsigned int nx) { m_nx = nx; }

        std::vector<double> GetVector() const;

        void SetStep(double delta);

        double GetStep() const;

    };

    // --------------------------------------------------------
    // FrHydroDB
    // --------------------------------------------------------

    class FrHydroDB_ {

    private:

        double m_gravityAcc;
        double m_waterDensity;
        double m_waterDepth;
        double m_normalizationLength;
        int m_nbody;

        std::vector<std::unique_ptr<FrBEMBody_>> m_bodies;
        std::unique_ptr<FrHydroMapper_> m_mapper;

        FrDiscretization1D_ m_frequencyDiscretization;
        FrDiscretization1D_ m_waveDirectionDiscretization;
        FrDiscretization1D_ m_timeDiscretization;

    public:

        FrHydroDB_() = default;

        explicit FrHydroDB_(std::string h5file);

        void ModeReader(FrHDF5Reader& reader, std::string path, FrBEMBody_* BEMBody);

        void ExcitationReader(FrHDF5Reader& reader, std::string path, FrBEMBody_* BEMBody);

        void RadiationReader(FrHDF5Reader& reader, std::string path, FrBEMBody_* BEMBody);

        void WaveDriftReader(FrHDF5Reader& reader, std::string path, FrBEMBody_* BEMBody);

        void HydrostaticReader(FrHDF5Reader& reader, std::string path, FrBEMBody_* BEMBody);

        unsigned int GetNbBodies() const { return (uint)m_bodies.size(); };

        void SetWaveDirectionDiscretization(const double minAngle, const double maxAngle, const unsigned int nbAngle);

        void SetTimeDiscretization(const double finalTime, const unsigned int nbTimeSamples);

        void SetFrequencyDiscretization(const double minFreq, const double maxFreq, const unsigned int nbFreq);

        std::vector<double> GetFrequencies() const { return m_frequencyDiscretization.GetVector(); }

        std::vector<double> GetWaveDirections(ANGLE_UNIT angleUnit, FRAME_CONVENTION fc) const;

        unsigned int GetNbFrequencies() const { return m_frequencyDiscretization.GetNbSample(); }

        double GetMaxFrequency() const { return m_frequencyDiscretization.GetMax(); }

        double GetMinFrequency() const { return m_frequencyDiscretization.GetMin(); }

        double GetStepFrequency() const {return m_frequencyDiscretization.GetStep(); }

        unsigned int GetNbWaveDirections() const { return m_waveDirectionDiscretization.GetNbSample(); }

        unsigned int GetNbTimeSamples() const { return m_timeDiscretization.GetNbSample(); }

        double GetFinalTime() const { return m_timeDiscretization.GetMax(); }

        double GetTimeStep() const { return m_timeDiscretization.GetStep(); }

        std::vector<double> GetTimeDiscretization() const { return m_timeDiscretization.GetVector(); }

        FrBEMBody_* NewBody(std::string bodyName);

        FrBEMBody_* GetBody(int ibody);

        FrBEMBody_* GetBody(std::shared_ptr<FrBody_> body);

        FrBEMBody_* GetBody(FrBody_* body);

        FrBody_* GetBody(FrBEMBody_* body);

        FrHydroMapper_* GetMapper();

        void Map(FrBEMBody_* BEMBody, FrBody_* body, std::shared_ptr<FrEquilibriumFrame_> eqFrame);

        void Map(int iBEMBody, FrBody_* body, std::shared_ptr<FrEquilibriumFrame_> eqFrame);

        std::vector<std::unique_ptr<FrBEMBody_>>::iterator begin() { return m_bodies.begin(); }

        std::vector<std::unique_ptr<FrBEMBody_>>::iterator end() { return m_bodies.end(); }
    };


    std::shared_ptr<FrHydroDB_> make_hydrodynamic_database(std::string h5file);

}  // end namespace frydom


#endif //FRYDOM_FRHYDRODB_H
