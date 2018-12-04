//
// Created by Lucas Letournel on 03/12/18.
//

#include "FrAiryIrregularWaveField.h"
#include <random>
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean_.h"
#include "frydom/environment/ocean/seabed/FrSeabed.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveDispersionRelation.h"
#include "frydom/environment/ocean/freeSurface/waves/FrKinematicStretching.h"

namespace frydom{

    FrAiryIrregularWaveField::FrAiryIrregularWaveField(FrFreeSurface_ *freeSurface) : FrWaveField_(freeSurface) {
        m_waveModel = LINEAR_WAVES;
        m_verticalFactor = std::make_unique<FrKinematicStretching_>();
        m_verticalFactor->SetInfDepth(m_infinite_depth);

        GenerateRandomWavePhases();

        m_waveSpectrum = std::make_unique<FrJonswapWaveSpectrum>();
    }

    void FrAiryIrregularWaveField::SetWaveFrequencies(double minFreq, double maxFreq, unsigned int nbFreq) {
        assert(minFreq>0);
        assert(minFreq<maxFreq);
        assert(nbFreq!=0);
        m_minFreq = minFreq;
        m_maxFreq = maxFreq;
        m_nbFreq = nbFreq;
        m_waveFrequencies = linspace(m_minFreq, m_maxFreq, m_nbFreq);

        // Caching wave numbers
        m_waveNumbers.clear();

        // Set the wave numbers, using the wave dispersion relation
        auto waterHeight = m_freeSurface->GetMeanHeight() - m_depth;
        auto gravityAcceleration = m_freeSurface->GetOcean()->GetEnvironment()->GetGravityAcceleration();
        m_waveNumbers = SolveWaveDispersionRelation(waterHeight, m_waveFrequencies, gravityAcceleration);

    }

    void FrAiryIrregularWaveField::SetMeanWaveDirection(double dirAngle, ANGLE_UNIT unit, FRAME_CONVENTION fc,
                                                        DIRECTION_CONVENTION dc) {
        // The wave direction angle is used internally with the convention NWU, GOTO, and RAD unit.
        m_meanDir = dirAngle;
        if (unit == DEG) m_meanDir *= DEG2RAD;
        if (IsNED(fc)) m_meanDir = - m_meanDir;
        if (IsCOMEFROM(dc)) m_meanDir -= MU_PI;

        mathutils::Normalize_0_2PI(m_meanDir);
    }

    void
    FrAiryIrregularWaveField::SetMeanWaveDirection(Direction direction, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
        assert(mathutils::IsClose(direction.Getuz(),0.));
        double dirAngle = atan2(direction.Getuy(),direction.Getux());
        SetMeanWaveDirection(dirAngle, RAD, fc, dc);
    }

    void FrAiryIrregularWaveField::SetDirectionalParameters(unsigned int nbDir, double spreadingFactor) {
        m_nbDir = nbDir;
        m_waveDirections.clear();

        assert(m_waveSpectrum!= nullptr);
        // TODO : gérer le cas où l'on souhaite un modèle d'étalement différent de cos2s
        m_waveSpectrum->SetCos2sDirectionaleModel(spreadingFactor);
        double dirBounds = 2.*acos(pow(1E-5,2.*spreadingFactor));
        m_waveDirections = linspace(m_meanDir - dirBounds, m_meanDir + dirBounds, nbDir);
    }

    void FrAiryIrregularWaveField::SetWaveSpectrum(WAVE_SPECTRUM_TYPE type) {
        m_waveSpectrum = MakeWaveSpectrum(type);
    }

    FrWaveSpectrum *FrAiryIrregularWaveField::GetWaveSpectrum() const { return m_waveSpectrum.get(); }

    void FrAiryIrregularWaveField::SetStretching(FrStretchingType type) {
        switch (type) {
            case NO_STRETCHING:
                m_verticalFactor = std::make_unique<FrKinematicStretching_>();
                break;
            case VERTICAL:
                m_verticalFactor = std::make_unique<FrKinStretchingVertical_>();
                break;
            case EXTRAPOLATE:
                m_verticalFactor = std::make_unique<FrKinStretchingExtrapol_>();
                break;
            case WHEELER:
                m_verticalFactor = std::make_unique<FrKinStretchingWheeler_>(this);
                break;
            case CHAKRABARTI:
                m_verticalFactor = std::make_unique<FrKinStretchingChakrabarti_>(this);
            case DELTA:
                m_verticalFactor = std::make_unique<FrKinStretchingDelta_>(this);
            default:
                m_verticalFactor = std::make_unique<FrKinematicStretching_>();
                break;
        }
        m_verticalFactor->SetInfDepth(m_infinite_depth);
    }


    void FrAiryIrregularWaveField::GenerateRandomWavePhases() {

        m_wavePhases = std::make_unique<std::vector<std::vector<double>>>();
        m_wavePhases->clear();
        m_wavePhases->reserve(m_waveDirections.size());

        std::vector<double> phases;
        phases.reserve(m_waveFrequencies.size());

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(0., MU_2PI);

        for (double dir: m_waveDirections) {
            phases.clear();
            for (double freq: m_waveFrequencies) {
                phases.push_back(dis(gen));
            }
            m_wavePhases->push_back(phases);
        }

    }

    double FrAiryIrregularWaveField::GetElevation(double x, double y) const {
        double elevation = 0.;

        double aik, ki, phi_ik;

        auto Amplitudes = m_waveSpectrum->GetWaveAmplitudes(m_waveFrequencies, m_waveDirections);
        double time = GetTime();
        for (unsigned int idir=0; idir<m_nbDir; ++idir) {
            double kdir = x*cos(m_waveDirections[idir]) + y*sin(m_waveDirections[idir]);
            for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
                aik = Amplitudes[idir][ifreq];
                ki = m_waveNumbers[ifreq];
                phi_ik = m_wavePhases->at(idir)[ifreq];
                elevation += aik * std::imag( exp(-JJ * (ki*kdir) + m_waveFrequencies[ifreq] * time - phi_ik ));
            }
        }
        return elevation;
    }

    void FrAiryIrregularWaveField::Initialize() {
        // Initialize wave frequency vector
        m_waveFrequencies = linspace(m_minFreq, m_maxFreq, m_nbFreq);

        // Caching wave numbers
        m_waveNumbers.clear();

        // Set the wave numbers, using the wave dispersion relation
        auto waterHeight = m_freeSurface->GetMeanHeight() - m_depth;
        auto gravityAcceleration = m_freeSurface->GetOcean()->GetEnvironment()->GetGravityAcceleration();
        m_waveNumbers = SolveWaveDispersionRelation(waterHeight, m_waveFrequencies, gravityAcceleration);

        // Checks wave phases, and randomly generate them if needed
        bool testSize = m_wavePhases->size() == m_nbDir;
        for (unsigned int idir=0; idir<m_nbDir; ++idir){
            testSize = testSize & m_wavePhases->at(idir).size() == m_nbFreq;
        }

        if (m_wavePhases==nullptr || !testSize) {
            GenerateRandomWavePhases();
        }

        FrWaveField_::Initialize();
        Update(0.);
    }


}
