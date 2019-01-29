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

        m_waveSpectrum = std::make_unique<FrJonswapWaveSpectrum_>();
    }

    void FrAiryIrregularWaveField::SetWaveFrequencies(double minFreq, double maxFreq, unsigned int nbFreq) {
        assert(minFreq>0);
        assert(minFreq<=maxFreq);
        assert(nbFreq!=0);
        m_minFreq = minFreq;
        m_maxFreq = maxFreq;
        m_nbFreq = nbFreq;

        m_waveFrequencies.clear();
        if (nbFreq ==1 ) {
            assert(minFreq==maxFreq);
            m_waveFrequencies.push_back(m_minFreq);
        }
        else {
            m_waveFrequencies = linspace(m_minFreq, m_maxFreq, m_nbFreq);
        }

        // Caching wave numbers
        m_waveNumbers.clear();

        // Set the wave numbers, using the wave dispersion relation
        auto gravityAcceleration = m_freeSurface->GetOcean()->GetEnvironment()->GetGravityAcceleration();

        double k;
        if (m_infinite_depth) {
            m_waveNumbers.reserve(m_nbFreq);
            for (auto freq:m_waveFrequencies) {
                k = pow(freq,2) / gravityAcceleration;
                m_waveNumbers.push_back(k);
            }
        }
        else {
            m_waveNumbers = SolveWaveDispersionRelation(m_freeSurface->GetOcean()->GetDepth(NWU), m_waveFrequencies,
                                                        gravityAcceleration);
        }
        if (!m_waveDirections.empty()){
            c_amplitude = m_waveSpectrum->GetWaveAmplitudes(m_waveFrequencies, m_waveDirections);
        }

    }

    void FrAiryIrregularWaveField::SetMeanWaveDirectionAngle(double dirAngle, ANGLE_UNIT unit, FRAME_CONVENTION fc,
                                                             DIRECTION_CONVENTION dc) {
        // The wave direction angle is used internally with the convention NWU, GOTO, and RAD unit.
        m_meanDir = dirAngle;
        if (unit == DEG) m_meanDir *= DEG2RAD;
        if (IsNED(fc)) m_meanDir = - m_meanDir;
        if (IsCOMEFROM(dc)) m_meanDir -= MU_PI;

        m_meanDir = mathutils::Normalize_0_2PI(m_meanDir);
        if (m_waveSpectrum->GetDirectionalModel() != nullptr) { ComputeWaveDirections(); }
    }

    void
    FrAiryIrregularWaveField::SetMeanWaveDirection(Direction direction, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
        assert(mathutils::IsClose(direction.Getuz(),0.));
        double dirAngle = atan2(direction.Getuy(),direction.Getux());
        SetMeanWaveDirectionAngle(dirAngle, RAD, fc, dc);
    }

    void FrAiryIrregularWaveField::SetDirectionalParameters(unsigned int nbDir, double spreadingFactor, WaveDirectionalModelType dirType) {
        m_nbDir = nbDir;
        m_waveDirections.clear();
        switch (dirType) {
            case NONE:
                m_waveSpectrum->SetDirectionalModel(dirType);
                break;
            case DIRTEST:
                m_waveSpectrum->SetDirectionalModel(dirType);
                break;
            case COS2S:
                m_waveSpectrum->SetCos2sDirectionalModel(spreadingFactor);
                break;
        }

        ComputeWaveDirections();
    }

    void FrAiryIrregularWaveField::ComputeWaveDirections(){

        assert(m_waveSpectrum!= nullptr);

        double spreadingFactor, dirBounds, BoundParameter;
        double theta_min, theta_max;

        m_waveDirections.clear();
        switch (m_waveSpectrum->GetDirectionalModel()->GetType()) {
            case NONE:
                m_waveDirections.push_back(m_meanDir);
                break;
            case COS2S:
                m_waveSpectrum->GetDirectionalModel()->GetDirectionBandwidth(theta_min, theta_max, m_meanDir);
                m_waveDirections = linspace(theta_min, theta_max, m_nbDir);
                break;
            case DIRTEST:
                m_waveDirections = linspace(m_meanDir, m_meanDir + 0.1, m_nbDir);
                break;
        }

        for (auto& dir:m_waveDirections) {dir = mathutils::Normalize_0_2PI(dir);};

        if (!m_waveFrequencies.empty()){
            c_amplitude = m_waveSpectrum->GetWaveAmplitudes(m_waveFrequencies, m_waveDirections);
        }
    }

    void FrAiryIrregularWaveField::SetWavePhases(std::vector<std::vector<double>> &wavePhases) {
        assert(wavePhases.size() == m_nbDir);
        for (auto& w: wavePhases) {
            assert(w.size() == m_nbFreq);
        }
        m_wavePhases = std::make_unique<std::vector<std::vector<double>>>(wavePhases);
    }

    FrJonswapWaveSpectrum_ *FrAiryIrregularWaveField::SetJonswapWaveSpectrum(double Hs, double Tp, FREQUENCY_UNIT unit, double gamma) {
        m_waveSpectrum = std::make_unique<FrJonswapWaveSpectrum_>(Hs,Tp,unit,gamma);
        return dynamic_cast<FrJonswapWaveSpectrum_*>(m_waveSpectrum.get());
    }

    FrPiersonMoskowitzWaveSpectrum_ *FrAiryIrregularWaveField::SetPiersonMoskovitzWaveSpectrum(double Hs, double Tp, FREQUENCY_UNIT unit) {
        m_waveSpectrum = std::make_unique<FrPiersonMoskowitzWaveSpectrum_>(Hs,Tp,unit);
        return dynamic_cast<FrPiersonMoskowitzWaveSpectrum_*>(m_waveSpectrum.get());
    }

    FrTestWaveSpectrum_ *FrAiryIrregularWaveField::SetTestWaveSpectrum() {
        m_waveSpectrum = std::make_unique<FrTestWaveSpectrum_>();
        return dynamic_cast<FrTestWaveSpectrum_*>(m_waveSpectrum.get());
    }

//    void FrAiryIrregularWaveField::SetWaveSpectrum(WAVE_SPECTRUM_TYPE type) {
//        m_waveSpectrum = MakeWaveSpectrum(type);
//    }

    FrWaveSpectrum_ *FrAiryIrregularWaveField::GetWaveSpectrum() const { return m_waveSpectrum.get(); }

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
            case HDELTA:
                m_verticalFactor = std::make_unique<FrKinStretchingHDelta_>(this);
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

    std::vector<double> FrAiryIrregularWaveField::GetWaveFrequencies(FREQUENCY_UNIT unit) const {
        std::vector<double> freqs = m_waveFrequencies;
        if (unit != RADS) {
            for (auto &freq: freqs) {
                freq = convert_frequency(freq, RADS, unit);
            }
        }
        return freqs;
    }

    std::vector<double> FrAiryIrregularWaveField::GetWaveDirections(ANGLE_UNIT unit, FRAME_CONVENTION fc,
                                                                    DIRECTION_CONVENTION dc) const {
        auto directions = m_waveDirections;

        if(IsNED(fc)) for (auto& dir: directions) {dir = -dir; }
        if(dc == COMEFROM) for (auto& dir: directions) { dir += M_PI; }
        for (auto& dir: directions) Normalize_0_2PI(dir);
        if(unit==DEG) for (auto& dir: directions) { dir *= MU_180_PI; }

        return directions;

    }

    void FrAiryIrregularWaveField::Initialize() {
        FrWaveField_::Initialize();
        m_verticalFactor->SetInfDepth(m_infinite_depth);

        if (m_waveDirections.empty()){m_waveDirections.push_back(m_meanDir);}

        if (m_minFreq==0. && m_maxFreq==0.) {
            GetWaveSpectrum()->GetFrequencyBandwidth(m_minFreq,m_maxFreq);
        }

        // Initialize wave frequency vector
        SetWaveFrequencies(m_minFreq, m_maxFreq, m_nbFreq);

        // Checks wave phases, and randomly generate them if needed
        bool testSize = true;
        if (m_wavePhases!=nullptr) {
            testSize = m_wavePhases->size() == m_nbDir;
            for (unsigned int idir = 0; idir < m_nbDir; ++idir) {
                testSize = testSize & m_wavePhases->at(idir).size() == m_nbFreq;
            }
        }

        if (m_wavePhases==nullptr || !testSize) {
            GenerateRandomWavePhases();
        }

        Update(0.);
    }

    std::vector<std::vector<Complex>>
    FrAiryIrregularWaveField::GetComplexElevation(double x, double y, FRAME_CONVENTION fc) const {
        double NWUsign = 1;
        if (IsNED(fc)) {y=-y; NWUsign = -NWUsign;}

        std::vector<std::vector<Complex>> ComplexElevation;
        ComplexElevation.reserve(m_nbDir);
        ComplexElevation.clear();

        std::vector<Complex> ComplexElevation_temp;
        ComplexElevation_temp.reserve(m_nbFreq);
        ComplexElevation_temp.clear();

        double aik, ki, phi_ik;
        Complex elevation;

        std::vector<double> amplitudeTemp;

        for (unsigned int idir=0; idir<m_nbDir; ++idir) {
            double kdir = x*cos(m_waveDirections[idir]) + y*sin(m_waveDirections[idir]);
            amplitudeTemp = c_amplitude[idir];
            ComplexElevation_temp.clear();
            for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
                ki = m_waveNumbers[ifreq];
                aik = amplitudeTemp[ifreq];
                phi_ik = m_wavePhases->at(idir)[ifreq];
                elevation = aik * exp(JJ * (ki * kdir - m_waveFrequencies[ifreq] * c_time - phi_ik) ) * NWUsign * c_ramp;
                ComplexElevation_temp.push_back(elevation);
            }
            ComplexElevation.push_back(ComplexElevation_temp);
        }

        return ComplexElevation;
    }

    std::vector<mathutils::Vector3d<Complex>>
    FrAiryIrregularWaveField::GetComplexVelocity(double x, double y, double z, FRAME_CONVENTION fc) const {
        double NWUsign = 1;
        if (IsNED(fc)) {y=-y; z=-z; NWUsign = -NWUsign;}

        std::vector<mathutils::Vector3d<Complex>> ComplexVel;
        ComplexVel.reserve(m_nbFreq);
        ComplexVel.clear();

        Complex Vx = 0, Vy = 0, Vz = 0;
        double ki, wi, thetaj;
        double Stretching, StretchingDZ;

        auto ComplexElevation = GetComplexElevation(x,y,fc);

        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
            ki = m_waveNumbers[ifreq];
            wi = m_waveFrequencies[ifreq];
            Vx = 0, Vy = 0, Vz = 0;
            Stretching = m_verticalFactor->Eval(x,y,z,ki,c_depth);
            StretchingDZ = m_verticalFactor->EvalDZ(x,y,z,ki,c_depth);
            for (unsigned int idir=0; idir<m_nbDir; ++idir) {
                thetaj = m_waveDirections[idir];
                Vx += cos(thetaj) * wi * ComplexElevation[idir][ifreq] * Stretching * NWUsign;
                Vy += sin(thetaj) * wi * ComplexElevation[idir][ifreq] * Stretching;
                Vz +=   - JJ / ki * wi * ComplexElevation[idir][ifreq] * StretchingDZ;
            }
            ComplexVel.emplace_back(Vx,Vy,Vz);
        }

        return ComplexVel;
    }

    double FrAiryIrregularWaveField::GetElevation(double x, double y, FRAME_CONVENTION fc) const {
        double elevation = 0.;

        auto ComplexElevation = GetComplexElevation(x,y, fc);

        for (unsigned int idir=0; idir<m_nbDir; ++idir) {
            for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
                elevation += std::imag(ComplexElevation[idir][ifreq]);
            }
        }
        return elevation;
    }

    Velocity FrAiryIrregularWaveField::GetVelocity(double x, double y, double z, FRAME_CONVENTION fc) const {
        Velocity Vel = {0.,0.,0.};
        auto cplxVel = GetComplexVelocity(x, y, z, fc);
        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
            Vel.GetVx() += std::imag(cplxVel[ifreq].x());
            Vel.GetVy() += std::imag(cplxVel[ifreq].y());
            Vel.GetVz() += std::imag(cplxVel[ifreq].z());
        }
        return Vel;
    }

    Acceleration FrAiryIrregularWaveField::GetAcceleration(double x, double y, double z, FRAME_CONVENTION fc) const {
        Acceleration Acc = {0.,0.,0.};
        auto cplxVel = GetComplexVelocity(x, y, z, fc);
        double wi;
        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
            wi = m_waveFrequencies[ifreq];
            Acc.GetAccX() += std::imag(-JJ * wi * cplxVel[ifreq].x());
            Acc.GetAccY() += std::imag(-JJ * wi * cplxVel[ifreq].y());
            Acc.GetAccZ() += std::imag(-JJ * wi * cplxVel[ifreq].z());
        }
        return Acc;
    }


//    Velocity FrAiryIrregularWaveField::GetVelocity(double x, double y, double z) const {
//        double Vx = 0, Vy = 0, Vz = 0;
//        double Stretching, StretchingDZ;
//        double ki, wi, thetaj;
//
//        auto ComplexElevation = GetComplexElevation(x,y);
//
//        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
//            ki = m_waveNumbers[ifreq];
//            wi = m_waveFrequencies[ifreq];
//            Stretching = m_verticalFactor->Eval(x,y,z,ki,c_depth);
//            StretchingDZ = m_verticalFactor->EvalDZ(x,y,z,ki,c_depth);
//            for (unsigned int idir=0; idir<m_nbDir; ++idir) {
//                thetaj = m_waveDirections[idir];
//                Vx += std::imag( cos(thetaj) * wi * ComplexElevation[idir][ifreq] * Stretching );
//                Vy += std::imag( sin(thetaj) * wi * ComplexElevation[idir][ifreq] * Stretching );
//                Vz += std::imag(   - JJ / ki * wi * ComplexElevation[idir][ifreq] * StretchingDZ);
//            }
//        }
//        return {Vx,Vy,Vz};
//    }
//
//    Acceleration FrAiryIrregularWaveField::GetAcceleration(double x, double y, double z) const {
//
//        double Ax = 0, Ay = 0, Az = 0;
//        double Stretching, StretchingDZ;
//        double ki, wi, thetaj;
//
//        auto ComplexElevation = GetComplexElevation(x,y);
//
//        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
//            ki = m_waveNumbers[ifreq];
//            wi = m_waveFrequencies[ifreq];
//            Stretching = m_verticalFactor->Eval(x,y,z,ki,c_depth);
//            StretchingDZ = m_verticalFactor->EvalDZ(x,y,z,ki,c_depth);
//            for (unsigned int idir=0; idir<m_nbDir; ++idir) {
//                thetaj = m_waveDirections[idir];
//                Ax += std::imag( - JJ * cos(thetaj) * wi * wi * ComplexElevation[idir][ifreq] * Stretching );
//                Ay += std::imag( - JJ * sin(thetaj) * wi * wi * ComplexElevation[idir][ifreq] * Stretching );
//                Az += std::imag( - 1.0 / ki * wi * wi * ComplexElevation[idir][ifreq] * StretchingDZ);
//            }
//        }
//        return {Ax,Ay,Az};
//    }

    double FrAiryIrregularWaveField::GetMeanWaveDirectionAngle(ANGLE_UNIT unit, FRAME_CONVENTION fc,
                                                               DIRECTION_CONVENTION dc) const {
        double dirAngle = m_meanDir;
        if (IsNED(fc)) dirAngle = - dirAngle;
        if (IsCOMEFROM(dc)) dirAngle -= MU_PI;
        if (unit == DEG) dirAngle *= RAD2DEG;

        return mathutils::Normalize_0_360(dirAngle);
    }

    Direction FrAiryIrregularWaveField::GetMeanWaveDirection(FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const {
        auto dirAngle = GetMeanWaveDirectionAngle(RAD, fc, dc);
        return {cos(dirAngle), sin(dirAngle), 0.};
    }

}
