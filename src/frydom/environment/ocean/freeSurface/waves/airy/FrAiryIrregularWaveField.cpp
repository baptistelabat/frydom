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

//        GenerateRandomWavePhases();

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

        m_meanDir = mathutils::Normalize_0_2PI(m_meanDir);
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
        double test = pow(1E-5,1.0/(2.*spreadingFactor));
        double dirBounds = 2.*acos(pow(1E-5,1.0/(2.*spreadingFactor)));
        m_waveDirections = linspace(m_meanDir - dirBounds, m_meanDir + dirBounds, nbDir);
        // FIXME : Normalize_0_2PI not working...
//        for (auto& dir:m_waveDirections) {dir = mathutils::Normalize_0_2PI(dir);};
//        for (unsigned int idir=0; idir<nbDir; ++idir) {
//            m_waveDirections[idir] = mathutils::Normalize_0_2PI(m_waveDirections[idir]);
//        };
    }

    void FrAiryIrregularWaveField::SetWavePhases(std::vector<std::vector<double>> &wavePhases) {
        assert(wavePhases.size() == m_nbDir);
        for (auto& w: wavePhases) {
            assert(w.size() == m_nbFreq);
        }
        m_wavePhases = std::make_unique<std::vector<std::vector<double>>>(wavePhases);
    }

    FrJonswapWaveSpectrum *FrAiryIrregularWaveField::SetJonswapWaveSpectrum(double Hs, double Tp, FREQUENCY_UNIT unit, double gamma) {
        m_waveSpectrum = std::make_unique<FrJonswapWaveSpectrum>(Hs,Tp,unit,gamma);
        return dynamic_cast<FrJonswapWaveSpectrum*>(m_waveSpectrum.get());
    }

    FrPiersonMoskowitzWaveSpectrum *FrAiryIrregularWaveField::SetPiersonMoskovitzWaveSpectrum(double Hs, double Tp, FREQUENCY_UNIT unit) {
        m_waveSpectrum = std::make_unique<FrPiersonMoskowitzWaveSpectrum>(Hs,Tp,unit);
        return dynamic_cast<FrPiersonMoskowitzWaveSpectrum*>(m_waveSpectrum.get());
    }

    FrTestWaveSpectrum *FrAiryIrregularWaveField::SetTestWaveSpectrum() {
        m_waveSpectrum = std::make_unique<FrTestWaveSpectrum>();
        return dynamic_cast<FrTestWaveSpectrum*>(m_waveSpectrum.get());
    }

//    void FrAiryIrregularWaveField::SetWaveSpectrum(WAVE_SPECTRUM_TYPE type) {
//        m_waveSpectrum = MakeWaveSpectrum(type);
//    }

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

    void FrAiryIrregularWaveField::Initialize() {
        FrWaveField_::Initialize();

        // Initialize wave frequency vector
        m_waveFrequencies = linspace(m_minFreq, m_maxFreq, m_nbFreq);

        // Caching wave numbers
        m_waveNumbers.clear();

        // Set the wave numbers, using the wave dispersion relation
        auto waterHeight = m_freeSurface->GetMeanHeight() - m_depth;
        auto gravityAcceleration = m_freeSurface->GetOcean()->GetEnvironment()->GetGravityAcceleration();
        m_waveNumbers = SolveWaveDispersionRelation(waterHeight, m_waveFrequencies, gravityAcceleration);

        // Checks wave phases, and randomly generate them if needed
        bool testSize = true;
        if (m_wavePhases!=nullptr) {
            testSize = m_wavePhases->size() == m_nbDir;
            for (unsigned int idir = 0; idir < m_nbDir; ++idir) {
                testSize = testSize & m_wavePhases->at(idir).size() == m_nbFreq;
            }
        }

        if (m_waveDirections.empty()){m_waveDirections.push_back(m_meanDir);}

        if (m_wavePhases==nullptr || !testSize) {
            GenerateRandomWavePhases();
        }

        Update(0.);
    }

    std::vector<std::vector<Complex>> FrAiryIrregularWaveField::GetComplexElevation(double x, double y) const {
        std::vector<std::vector<Complex>> ComplexElevation;
        ComplexElevation.reserve(m_nbDir);
        ComplexElevation.clear();

        std::vector<Complex> ComplexElevation_temp;
        ComplexElevation_temp.reserve(m_nbFreq);
        ComplexElevation_temp.clear();

        double aik, ki, phi_ik;
        Complex elevation;

        auto Amplitudes = m_waveSpectrum->GetWaveAmplitudes(m_waveFrequencies, m_waveDirections);
        double time = GetTime();
        for (unsigned int idir=0; idir<m_nbDir; ++idir) {
            double kdir = x*cos(m_waveDirections[idir]) + y*sin(m_waveDirections[idir]);
            for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
                ki = m_waveNumbers[ifreq];
                aik = Amplitudes[idir][ifreq];
                phi_ik = m_wavePhases->at(idir)[ifreq];
                elevation = aik * exp(-JJ * (ki * kdir + m_waveFrequencies[ifreq] * time - phi_ik) );
                ComplexElevation_temp.push_back(elevation);
            }
            ComplexElevation.push_back(ComplexElevation_temp);
        }

        return ComplexElevation;
    }

    double FrAiryIrregularWaveField::GetElevation(double x, double y) const {
        double elevation = 0.;

        auto ComplexElevation = GetComplexElevation(x,y);

        for (unsigned int idir=0; idir<m_nbDir; ++idir) {
            for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
                elevation += std::imag(ComplexElevation[idir][ifreq]);
            }
        }


//        double aik, ki, phi_ik;
//
//        auto Amplitudes = m_waveSpectrum->GetWaveAmplitudes(m_waveFrequencies, m_waveDirections);
//        double time = GetTime();
//        for (unsigned int idir=0; idir<m_nbDir; ++idir) {
//            double kdir = x*cos(m_waveDirections[idir]) + y*sin(m_waveDirections[idir]);
//            for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
//                aik = Amplitudes[idir][ifreq];
//                ki = m_waveNumbers[ifreq];
//                phi_ik = m_wavePhases->at(idir)[ifreq];
//                elevation += aik * std::imag( exp(-JJ * (ki*kdir) + m_waveFrequencies[ifreq] * time - phi_ik ));
//            }
//        }
        return elevation;
    }

    std::vector<mathutils::Vector3d<Complex>> FrAiryIrregularWaveField::GetComplexVelocity(double x, double y, double z) const {

        std::vector<mathutils::Vector3d<Complex>> ComplexVel;
        ComplexVel.reserve(m_nbFreq);
        ComplexVel.clear();

        Complex Vx = 0, Vy = 0, Vz = 0;
        double ki, wi, thetaj;
        double Stretching, StretchingDZ;

        auto ComplexElevation = GetComplexElevation(x,y);

        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
            ki = m_waveNumbers[ifreq];
            wi = m_waveFrequencies[ifreq];
            Vx = 0, Vy = 0, Vz = 0;
            Stretching = m_verticalFactor->Eval(x,y,z,ki,m_depth);
            StretchingDZ = m_verticalFactor->EvalDZ(x,y,z,ki,m_depth);
            for (unsigned int idir=0; idir<m_nbDir; ++idir) {
                thetaj = m_waveDirections[idir];
                Vx += cos(thetaj) * wi * ComplexElevation[idir][ifreq] * Stretching;
                Vy += sin(thetaj) * wi * ComplexElevation[idir][ifreq] * Stretching;
                Vz +=   - JJ / ki * wi * ComplexElevation[idir][ifreq] * StretchingDZ;
            }
            ComplexVel.emplace_back(Vx,Vy,Vz);
        }

        return ComplexVel;
    }

//    Velocity FrAiryIrregularWaveField::GetVelocity(double x, double y, double z) const {
//        Velocity Vel = {0.,0.,0.};
//        auto cplxVel = GetComplexVelocity(x, y, z);
//        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
//            Vel.GetVx() += std::imag(cplxVel[ifreq].x());
//            Vel.GetVy() += std::imag(cplxVel[ifreq].y());
//            Vel.GetVz() += std::imag(cplxVel[ifreq].z());
//        }
//        return Vel;
//    }
//
//    Acceleration FrAiryIrregularWaveField::GetAcceleration(double x, double y, double z) const {
//        Acceleration Acc = {0.,0.,0.};
//        auto cplxVel = GetComplexVelocity(x, y, z);
//        double wi;
//        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
//            wi = m_waveFrequencies[ifreq];
//            Acc.GetAccX() += std::imag(-JJ * wi * cplxVel[ifreq].x());
//            Acc.GetAccY() += std::imag(-JJ * wi * cplxVel[ifreq].y());
//            Acc.GetAccZ() += std::imag(-JJ * wi * cplxVel[ifreq].z());
//        }
//        return Acc;
//    }


    Velocity FrAiryIrregularWaveField::GetVelocity(double x, double y, double z) const {
        double Vx = 0, Vy = 0, Vz = 0;
        double Stretching, StretchingDZ;
        double ki, wi, thetaj;

        auto ComplexElevation = GetComplexElevation(x,y);

        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
            ki = m_waveNumbers[ifreq];
            wi = m_waveFrequencies[ifreq];
            Stretching = m_verticalFactor->Eval(x,y,z,ki,m_depth);
            StretchingDZ = m_verticalFactor->EvalDZ(x,y,z,ki,m_depth);
            for (unsigned int idir=0; idir<m_nbDir; ++idir) {
                thetaj = m_waveDirections[idir];
                Vx += std::imag( cos(thetaj) * wi * ComplexElevation[idir][ifreq] * Stretching );
                Vy += std::imag( sin(thetaj) * wi * ComplexElevation[idir][ifreq] * Stretching );
                Vz += std::imag(   - JJ / ki * wi * ComplexElevation[idir][ifreq] * StretchingDZ);
            }
        }
        return {Vx,Vy,Vz};
    }

    Acceleration FrAiryIrregularWaveField::GetAcceleration(double x, double y, double z) const {

        double Ax = 0, Ay = 0, Az = 0;
        double Stretching, StretchingDZ;
        double ki, wi, thetaj;

        auto ComplexElevation = GetComplexElevation(x,y);

        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
            ki = m_waveNumbers[ifreq];
            wi = m_waveFrequencies[ifreq];
            Stretching = m_verticalFactor->Eval(x,y,z,ki,m_depth);
            StretchingDZ = m_verticalFactor->EvalDZ(x,y,z,ki,m_depth);
            for (unsigned int idir=0; idir<m_nbDir; ++idir) {
                thetaj = m_waveDirections[idir];
                Ax += std::imag( - JJ * cos(thetaj) * wi * wi * ComplexElevation[idir][ifreq] * Stretching );
                Ay += std::imag( - JJ * sin(thetaj) * wi * wi * ComplexElevation[idir][ifreq] * Stretching );
                Az += std::imag( - 1.0 / ki * wi * wi * ComplexElevation[idir][ifreq] * StretchingDZ);
            }
        }
        return {Ax,Ay,Az};
    }

}
