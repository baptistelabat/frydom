//
// Created by frongere on 31/10/17.
//

#include "FrWaveField.h"

#include <random>

#include "frydom/core/functions/FrRamp.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/FrOcean_.h"
#include "frydom/environment/ocean/seabed/FrSeabed.h"

#include "FrWaveProbe.h"
#include "FrFlowSensor.h"
#include "FrWaveDispersionRelation.h"

#include "FrWaveProbe.h"

#include "frydom/utils/FrUtils.h"

namespace frydom {


//    // FrRamp definitions
//
//    void FrRamp::SetDuration(double duration) {
//        m_t1 = m_t0 + duration;
//    }
//
//    void FrRamp::SetIncrease() {
//        m_increasing = true;
//        Initialize();
//    }
//
//    void FrRamp::SetDecrease() {
//        m_increasing = false;
//        Initialize();
//    }
//
//    void FrRamp::SetMinVal(double minVal) { m_min = minVal; }
//
//    void FrRamp::SetMaxVal(double maxVal) { m_max = maxVal; }
//
//    bool FrRamp::IsActive() {
//        return m_active;
//    }
//
//    void FrRamp::Deactivate() {
//        m_active = false;
//    }
//
//    void FrRamp::Initialize() {
//        double y0, y1;
//
//        if (m_increasing) {
//            y0 = m_min;
//            y1 = m_max;
//        } else {
//            y0 = m_max;
//            y1 = m_min;
//        }
//        c_a = (y1 - y0) / (m_t1 - m_t0);
//        c_b = y0 - c_a * m_t0;
//    }
//
//    void FrRamp::Apply(const double t, double &value) {
//
//        if (!m_active) {
//            return;
//        }
//
//        double y0, y1;
//        if (m_increasing) {
//            y0 = m_min;
//            y1 = m_max;
//        } else {
//            y0 = m_max;
//            y1 = m_min;
//        }
//
//
//        if (t < m_t0) {
//            value *= y0;
//            return;
//        }
//
//        if (t <= m_t1) {
//            value *= c_a * t + c_b;
//            return;
//        }
//
//        value *= y1;
//
//    }
//
//    void FrRamp::Apply(const double t, chrono::ChVector<double> &vect) {
//        Apply(t, vect.x());
//        Apply(t, vect.y());
//        Apply(t, vect.z());
//    }
//
//    void FrRamp::StepFinalize() {}


    // FrWaveField definitions

    FrWaveField::FrWaveField(FrFreeSurface *freeSurface) : m_freeSurface(freeSurface) {

    }

    void FrWaveField::Update(double time) {
        m_time = time;
    }

    double FrWaveField::GetTime() const { return m_time; }

    void FrWaveField::SetWaveSpectrum(WAVE_SPECTRUM_TYPE type) {
        m_waveSpectrum = MakeWaveSpectrum(type);
    }

    WAVE_MODEL FrWaveField::GetWaveModel() const { return m_waveModel; }

    std::shared_ptr<FrRamp_> FrWaveField::GetWaveRamp() const {
        return m_waveRamp;
    }

    chrono::ChVector<double> FrWaveField::GetVelocity(double x, double y, double z, bool cutoff) const {

        if (cutoff) {
            auto wave_elevation = GetElevation(x, y);
            if (wave_elevation < z) {
                return chrono::VNULL;
            }
        }
        return GetVelocity(x, y, z);
    }

    chrono::ChVector<double> FrWaveField::GetVelocity(chrono::ChVector<double> vect) const {
        return GetVelocity(vect.x(), vect.y(), vect.z());
    }

    chrono::ChVector<double> FrWaveField::GetAcceleration(double x, double y, double z, bool cutoff) const {

        if (cutoff) {
            auto wave_elevation = GetElevation(x, y);
            if (wave_elevation < z) {
                return chrono::VNULL;
            }
        }
        return GetAcceleration(x, y, z);
    }

    chrono::ChVector<double> FrWaveField::GetAcceleration(chrono::ChVector<double> vect) const {
        return GetAcceleration(vect.x(), vect.y(), vect.z());
    }

    void FrWaveField::Initialize() {}

    void FrWaveField::StepFinalize() {}



    // FrNullWaveField definitions

    FrNullWaveField::FrNullWaveField(FrFreeSurface* freeSurface) : FrWaveField(freeSurface) {}

    double FrNullWaveField::GetElevation(double x, double y) const {
        return 0.;
    }

    chrono::ChVector<double> FrNullWaveField::GetVelocity(double x, double y, double z) const {
        return chrono::ChVector<double>(0.);
    }

    chrono::ChVector<double> FrNullWaveField::GetAcceleration(double x, double y, double z) const {
        return chrono::ChVector<double>(0.);
    }

    std::vector<std::vector<double>>
    FrNullWaveField::GetElevation(const std::vector<double> &xVect, const std::vector<double> &yVect) const {

        auto nx = xVect.size();
        auto ny = yVect.size();

        std::vector<std::vector<double>> elevations;
        elevations.reserve(nx);

        std::vector<double> elev;
        elev.reserve(ny);
        for (unsigned int iy=0; iy<ny; ++iy) {
            elev.push_back(0.);
        }

        for (unsigned int ix=0; ix<nx; ++ix) {
            elevations.push_back(elev);
        }

        return elevations;

    }

    std::vector<std::vector<std::vector<chrono::ChVector<double>>>>
    FrNullWaveField::GetVelocityGrid(const std::vector<double> &xvect, const std::vector<double> &yvect,
                                     const std::vector<double> &zvect) const {

        auto nx = xvect.size();
        auto ny = yvect.size();
        auto nz = zvect.size();

        std::vector<std::vector<std::vector<chrono::ChVector<double>>>> velocity;
        std::vector<std::vector<chrono::ChVector<double>>> velocity_x;
        std::vector<chrono::ChVector<double>> velocity_y;
        chrono::ChVector<double> velocity_z;

        velocity_y.reserve(nz);
        for (unsigned int i=0; i<nz; ++i) {
            velocity_y.push_back(0.);
        }

        velocity_x.reserve(ny);
        for (unsigned int i=0; i<ny; ++i) {
            velocity_x.push_back(velocity_y);
        }

        velocity.reserve(nx);
        for (unsigned int i=0; i<nx; ++i) {
            velocity.push_back(velocity_x);
        }

        return velocity;
    }

    std::shared_ptr<FrWaveProbe> FrNullWaveField::NewWaveProbe(double x, double y) {
        // TODO
    }


    // FrLinearWaveField definitions

    std::shared_ptr<FrLinearWaveProbe> FrLinearWaveField::NewWaveProbe(double x, double y) {
        auto waveProbe = std::make_shared<FrLinearWaveProbe>(this, x, y);
//        waveProbe->SetWaveField(this);
        m_waveProbes.push_back(waveProbe);
        return waveProbe;
    }

//    void FrLinearWaveField::AddWaveProbe(std::shared_ptr<FrLinearWaveProbe> waveProbe) {
//        waveProbe->SetWaveField(this);
//        m_waveProbes.push_back(waveProbe);
//    }

    std::shared_ptr<FrLinearWaveProbe> FrLinearWaveField::NewWaveProbe() {
        auto waveProbe = std::make_shared<FrLinearWaveProbe>(this);
//        waveProbe->SetWaveField(this);
        m_waveProbes.push_back(waveProbe);
        return waveProbe;
    }

    std::shared_ptr<FrLinearFlowSensor> FrLinearWaveField::NewFlowSensor(double x, double y, double z) {
        auto flowSensor = std::make_shared<FrLinearFlowSensor>(this, x, y, z);
        m_flowSensor.push_back(flowSensor);
        return flowSensor;
    }

    std::vector<std::vector<double>> FrLinearWaveField::_GetWaveAmplitudes() const {
        std::vector<std::vector<double>> waveAmplitudes;
        std::vector<double> ampl;
        switch (m_linearWaveType) {

            case LINEAR_REGULAR:
                ampl.push_back(m_height * 0.5);
                waveAmplitudes.push_back(ampl);
                break;

            case LINEAR_IRREGULAR:
                ampl = m_waveSpectrum->GetWaveAmplitudes(m_nbFreq, m_minFreq, m_maxFreq);
                waveAmplitudes.push_back(ampl);
                break;

            case LINEAR_DIRECTIONAL:
                waveAmplitudes = m_waveSpectrum->GetWaveAmplitudes(m_nbFreq, m_minFreq, m_maxFreq,
                                                                   m_nbDir, m_minDir, m_maxDir, m_meanDir);
                break;
        }
        return waveAmplitudes;
    }


    void FrLinearWaveField::SetStretching(FrStretchingType type) {

        switch (type) {
            case NO_STRETCHING:
                m_verticalFactor = std::make_shared<FrKinematicStretching>();
                break;
            case VERTICAL:
                m_verticalFactor = std::make_shared<FrKinStretchingVertical>();
                break;
            case EXTRAPOLATE:
                m_verticalFactor = std::make_shared<FrKinStretchingExtrapol>();
                break;
            case WHEELER:
                m_verticalFactor = std::make_shared<FrKinStretchingWheeler>(this);
                break;
            case CHAKRABARTI:
                m_verticalFactor = std::make_shared<FrKinStretchingChakrabarti>(this);
            case DELTA:
                m_verticalFactor = std::make_shared<FrKinStretchingDelta>(this);
            default:
                m_verticalFactor = std::make_shared<FrKinematicStretching>();
                break;
        }
        m_verticalFactor->SetInfDepth(m_infinite_depth);
    }

    FrLinearWaveField::FrLinearWaveField(FrFreeSurface* freeSurface, LINEAR_WAVE_TYPE type) : FrWaveField(freeSurface) {

        SetType(type);
        GenerateRandomWavePhases();
        Initialize();
        Update(0.);

        m_waveRamp = std::make_shared<FrRamp_>();
//        m_waveRamp->
        m_waveRamp->Initialize();

        m_verticalFactor = std::make_shared<FrKinematicStretching>();
        m_verticalFactor->SetInfDepth(m_infinite_depth);

    }

    LINEAR_WAVE_TYPE FrLinearWaveField::GetType() const {
        return m_linearWaveType;
    }

    void FrLinearWaveField::SetType(LINEAR_WAVE_TYPE type) {
        m_linearWaveType = type;

        switch (type) {
            case LINEAR_REGULAR:
                m_waveSpectrum = nullptr;
                m_nbFreq = 1;
                m_nbDir = 1;
                m_minDir = m_meanDir;
                m_maxDir = m_meanDir;
                break;
            case LINEAR_IRREGULAR:
                m_nbDir = 1;
                m_minDir = m_meanDir;
                m_maxDir = m_meanDir;

                // Creating a default waveSpectrum
                SetWaveSpectrum(JONSWAP);
                break;
            case LINEAR_DIRECTIONAL:

                break;
        }
    }

    void FrLinearWaveField::SetRegularWaveHeight(double height) {
        m_height = height;
    }

    void FrLinearWaveField::SetRegularWavePeriod(double period, FREQUENCY_UNIT unit) {
        m_period = convert_frequency(period, unit, S);
        Initialize();
    }

    unsigned int FrLinearWaveField::GetNbFrequencies() const { return m_nbFreq; }

    double FrLinearWaveField::GetMinFrequency() const { return m_minFreq; }

    double FrLinearWaveField::GetMaxFrequency() const { return m_maxFreq; }

    unsigned int FrLinearWaveField::GetNbWaveDirections() const { return m_nbDir; }

    double FrLinearWaveField::GetMinWaveDirection() const { return m_minDir; }

    double FrLinearWaveField::GetMaxWaveDirection() const { return m_maxDir; }

    double FrLinearWaveField::GetMeanWaveDirection(ANGLE_UNIT unit) const {
        double meanWaveDir = m_meanDir;
        if (unit == DEG) {
            meanWaveDir *= MU_180_PI;
        }
        return meanWaveDir;
    }

    void FrLinearWaveField::SetMeanWaveDirection(const double meanDirection, ANGLE_UNIT unit) {
        if (unit == DEG) {
            m_meanDir = meanDirection * MU_PI_180;
        } else {
            m_meanDir = meanDirection;
        }

        if (!(m_linearWaveType == LINEAR_DIRECTIONAL)) {
            m_minDir = m_meanDir;
            m_maxDir = m_meanDir;
        }
    }

    std::vector<double> FrLinearWaveField::GetWaveDirections(ANGLE_UNIT unit) const {
        std::vector<double> waveDirections;

        if (m_linearWaveType != LINEAR_DIRECTIONAL) {
            waveDirections.push_back(GetMeanWaveDirection(unit));

        } else {
            if (unit == DEG) {
                waveDirections = linspace(m_minDir * MU_PI_180, m_maxDir * MU_PI_180, m_nbDir);
            } else {
                waveDirections = linspace(m_minDir, m_maxDir, m_nbDir);
            }

        }
        return waveDirections;
    }

    void FrLinearWaveField::SetWaveDirections(const double minDir, const double maxDir, const unsigned int nbDir,
                                              ANGLE_UNIT unit) {

        if (unit == DEG) {
            m_minDir = minDir * MU_PI_180;
            m_maxDir = maxDir * MU_PI_180;
        } else {
            m_minDir = minDir;
            m_maxDir = maxDir;
        }
        m_nbDir = nbDir;

        if (m_nbDir == 1) {
            if (m_nbFreq == 1) {
                SetType(LINEAR_REGULAR);
            } else {
                SetType(LINEAR_IRREGULAR);
            }
        }
    }

    std::vector<double> FrLinearWaveField::GetWaveFrequencies(FREQUENCY_UNIT unit) const { return c_waveFrequencies; }

    void FrLinearWaveField::SetWavePulsations(const double minFreq, const double maxFreq, const unsigned int nbFreq,
                                              FREQUENCY_UNIT unit) {

        m_minFreq = convert_frequency(minFreq, unit, RADS);
        m_maxFreq = convert_frequency(maxFreq, unit, RADS);
        m_nbFreq = nbFreq;

        if (nbFreq == 1) {
            SetType(LINEAR_REGULAR);
        }

        Initialize();
    }

    void FrLinearWaveField::Initialize() {
        // TODO: aller chercher les infos dans FrOffshoreSystem

        // Caching wave frequencies
        c_waveFrequencies.clear();

        if (m_linearWaveType == LINEAR_REGULAR) {
            c_waveFrequencies.push_back(S2RADS(m_period));
        } else {
            c_waveFrequencies = linspace(m_minFreq, m_maxFreq, m_nbFreq);
        }

        // Caching wave numbers
        c_waveNumbers.clear();

        // waterHeight = environment->GetDepth()
        // grav = environment->GetGrav()
        // TODO: charger le hauteur d'eau et la gravite depuis
        double waterHeight = 10.;
        double grav = 9.81;
        c_waveNumbers = SolveWaveDispersionRelation(waterHeight, c_waveFrequencies, grav);

        if (m_wavePhases==nullptr) {
            this->GenerateRandomWavePhases();
        }

        m_waveRamp->Initialize();

        FrWaveField::Initialize();
        Update(0.);
    }

    std::vector<std::vector<double>>* FrLinearWaveField::GetWavePhases() const {
        return m_wavePhases.get();
    }

    void FrLinearWaveField::SetWavePhases(std::vector<std::vector<double>> &wavePhases) {
        assert(wavePhases.size() == m_nbDir);
        for (auto& w: wavePhases) {
            assert(w.size() == m_nbFreq);
        }
        m_wavePhases = std::make_unique<std::vector<std::vector<double>>>(wavePhases);
    }

    void FrLinearWaveField::GenerateRandomWavePhases() {

        m_wavePhases = std::make_unique<std::vector<std::vector<double>>>();

        m_wavePhases->clear();
        m_wavePhases->reserve(m_nbDir);

        std::vector<double> phases;
        phases.reserve(m_nbFreq);

        if (m_linearWaveType == LINEAR_REGULAR) {
            phases.push_back(0.);
            m_wavePhases->push_back(phases);
        } else {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<double> dis(0., MU_2PI);

            for (uint idir=0; idir<m_nbDir; ++idir) {
                phases.clear();
                for (uint iw=0; iw<m_nbFreq; ++iw) {
                    phases.push_back(dis(gen));
                }
                m_wavePhases->push_back(phases);
            }
        }
    }

    std::vector<double> FrLinearWaveField::GetWaveLengths() const {
        // TODO
    }

    std::vector<double> FrLinearWaveField::GetWaveNumbers() const {
        return c_waveNumbers;
    }

    std::vector<std::vector<std::complex<double>>>
    FrLinearWaveField::GetCmplxElevation(const double x, const double y, bool steady) const {


        std::vector<std::vector<std::complex<double>>> cmplxElevations;  // is nbDir x nbFreq
        cmplxElevations.reserve(m_nbDir);
        std::vector<std::complex<double>> elev;
        elev.reserve(m_nbFreq);

        std::vector<double> w_;
        w_.reserve(m_nbDir);
        if (m_linearWaveType == LINEAR_DIRECTIONAL) {
            auto wave_dirs = GetWaveDirections(RAD);
            for (auto wave_dir: wave_dirs) {
                w_.push_back(x * cos(wave_dir) + y * sin(wave_dir));
            }
        } else {
            w_.push_back(x * cos(m_meanDir) + y * sin(m_meanDir));
        }

        std::vector<std::vector<double>> waveAmplitudes = _GetWaveAmplitudes();

        std::complex<double> aik, val;
        double ki, wi, wk_, phi_ik;

        for (unsigned int idir=0; idir<m_nbDir; ++idir) {
            elev.clear();
            wk_ = w_[idir];

            for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
                aik = waveAmplitudes[idir][ifreq];
                ki = c_waveNumbers[ifreq];  // FIXME: Ici, on doit avoir le nombre d'onde
                phi_ik = m_wavePhases->at(idir)[ifreq];

                val = aik * exp(JJ * (ki*wk_ + phi_ik));

                if (!steady) {  // TODO : utiliser les valeurs mises en cache...
                    val *= c_emjwt[ifreq];
                }

                elev.push_back(val);
            }
            cmplxElevations.push_back(elev);
        }
        return cmplxElevations;

    }

    std::vector<chrono::ChVector<std::complex<double>>>
    FrLinearWaveField::GetSteadyVelocity(const double x, const double y, const double z) const {

        chrono::ChVector<std::complex<double>> vdiff;

        // Initialization
        std::vector<chrono::ChVector<std::complex<double>>> steadyVelocity;
        steadyVelocity.reserve(m_nbFreq);
        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
            steadyVelocity.emplace_back(0.);
        }

        // Get the complex wave elevation
        auto cmplxElevations = GetCmplxElevation(x, y, true);

        // Wave direction
        std::vector<double> angles;
        angles.reserve(m_nbDir);
        if (m_linearWaveType == LINEAR_DIRECTIONAL) {
            angles = GetWaveDirections(RAD);
        } else {
            angles.push_back(m_meanDir);
        }

        // Pre-compute direction coefficient
        std::vector<std::complex<double>> cxi, cyi;
        for (auto& angle: angles) {
            cxi.push_back(JJ*cos(angle));
            cyi.push_back(JJ*sin(angle));
        }

        // Compute steady velocity
        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {

            if (m_verticalFactor->IsSteady()) {
                vdiff.x() = cos(m_meanDir) * JJ * c_waveNumbers[ifreq] * m_verticalFactor->Eval(x, y, z, c_waveNumbers[ifreq], m_depth);
                vdiff.y() = sin(m_meanDir) * JJ * c_waveNumbers[ifreq] * m_verticalFactor->Eval(x, y, z, c_waveNumbers[ifreq], m_depth);
                vdiff.z() = m_verticalFactor->EvalDZ(x, y, z, c_waveNumbers[ifreq], m_depth);
            } else {
                vdiff = chrono::ChVector<double>(1.);
            }

            vdiff *= c_waveFrequencies[ifreq] / c_waveNumbers[ifreq];

            for (unsigned int idir=0; idir<angles.size(); ++idir) {

                steadyVelocity[ifreq].x() += vdiff.x() * cmplxElevations[idir][ifreq];
                steadyVelocity[ifreq].y() += vdiff.y() * cmplxElevations[idir][ifreq];
                steadyVelocity[ifreq].z() += vdiff.z() * cmplxElevations[idir][ifreq];

            }
        }

        return steadyVelocity;

    }

    std::vector<std::complex<double>> FrLinearWaveField::GetSteadyElevation(const double x, const double y) const {
        auto cmplxElevation = GetCmplxElevation(x, y, true);

        std::vector<std::complex<double>> steadyElevation;
        steadyElevation.reserve(m_nbFreq);
        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
            steadyElevation.emplace_back(0.);
        }

        // TODO: continuer
        for (unsigned int idir=0; idir<m_nbDir; ++idir) {
            for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
                steadyElevation[ifreq] += cmplxElevation[idir][ifreq];
            }
        }
        return steadyElevation;

    }

    FrWaveSpectrum *FrLinearWaveField::GetWaveSpectrum() const { return m_waveSpectrum.get(); }

    void FrLinearWaveField::SetReturnPeriod() {
        // TODO
    }

    double FrLinearWaveField::GetReturnPeriod() const {
        // TODO
    }

    void FrLinearWaveField::Update(double time) {
        m_time = time;

        // Updating exp(-jwt)
        std::vector<double> w = GetWaveFrequencies(RADS);
        c_emjwt.clear();
        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
            c_emjwt.push_back(exp(-JJ*w[ifreq]*time));
        }

    }

    const std::vector<std::complex<double>> &FrLinearWaveField::GetTimeCoeffs() const {
        return c_emjwt;
    }

    const std::vector<std::complex<double>> &FrLinearWaveField::GetTimeCoeffs(chrono::ChVector<double> vel) const {



    }

    std::vector<std::complex<double>> FrLinearWaveField::GetTimeCoeffsDt() const {

        std::vector<std::complex<double>> emjwtdt;
        std::vector<double> w = GetWaveFrequencies(RADS);

        emjwtdt.reserve(m_nbFreq);
        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
            emjwtdt.push_back( -JJ * w[ifreq] * c_emjwt[ifreq] );
        }
        return emjwtdt;
    }

    double FrLinearWaveField::GetElevation(double x, double y) const {
        // FIXME: appliquer la rampe ici aussi !!!
        auto steadyElevation =  GetSteadyElevation(x, y);

        double elevation = 0.;
        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
            elevation += std::imag( steadyElevation[ifreq] * c_emjwt[ifreq] );
        }
        return elevation;
    }

    chrono::ChVector<double> FrLinearWaveField::GetVelocity(double x, double y, double z) const {

        auto steadyVelocity = GetSteadyVelocity(x, y, z);

        std::vector<double> Kz(m_nbFreq, 1.), dKz(m_nbFreq, 1.);
        if (!m_verticalFactor->IsSteady()) {
            Kz = m_verticalFactor->Eval(x, y, z, c_waveNumbers, m_depth);
            dKz = m_verticalFactor->EvalDZ(x, y, z, c_waveNumbers, m_depth);
        }

        chrono::ChVector<double> velocity = 0.;
        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
            velocity.x() += std::real( Kz[ifreq] * steadyVelocity[ifreq].x() * c_emjwt[ifreq]);
            velocity.y() += std::real( Kz[ifreq] * steadyVelocity[ifreq].y() * c_emjwt[ifreq]);
            velocity.z() += std::real( dKz[ifreq] * steadyVelocity[ifreq].z() * c_emjwt[ifreq]);
        }
        return velocity;
    }

    chrono::ChVector<double> FrLinearWaveField::GetAcceleration(double x, double y, double z) const {

        auto steadyVelocity = GetSteadyVelocity(x, y, z);
        auto emjwt_dt = this->GetTimeCoeffsDt();

        chrono::ChVector<std::complex<double>> acceleration(0.);
        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
            acceleration += steadyVelocity[ifreq] * emjwt_dt[ifreq];
        }

        chrono::ChVector<double> realAcceleration = ChReal(acceleration);

        if (m_waveRamp && m_waveRamp->IsActive()) {
            realAcceleration *= m_waveRamp->Get_y(m_time);
        }

        return realAcceleration;
    }

    std::vector<std::vector<double>>
    FrLinearWaveField::GetElevation(const std::vector<double> &xVect, const std::vector<double> &yVect) const {

        auto nx = xVect.size();
        auto ny = yVect.size();

        std::vector<std::vector<double>> elevations;
        std::vector<double> elev;

        elevations.reserve(nx);
        elev.reserve(ny);

        double eta, x, y;
        for (unsigned int ix=0; ix<nx; ++ix) {
            elev.clear();
            x = xVect[ix];
            for (unsigned int iy=0; iy<ny; ++iy) {
                y = yVect[iy];
                eta = GetElevation(x, y);
                elev.push_back(eta);
            }
            elevations.push_back(elev);
        }
        return elevations;
    }

    std::vector<std::vector<std::vector<chrono::ChVector<double>>>>
    FrLinearWaveField::GetVelocityGrid(const std::vector<double> &xvect, const std::vector<double> &yvect,
                                       const std::vector<double> &zvect) const {

        auto nx = xvect.size();
        auto ny = yvect.size();
        auto nz = zvect.size();

        std::vector<std::vector<std::vector<chrono::ChVector<double>>>> velocity;
        std::vector<std::vector<chrono::ChVector<double>>> velocity_x;
        std::vector<chrono::ChVector<double>> velocity_y;
        chrono::ChVector<double> velocity_z;

        double x, y, z;

        for (unsigned int ix=0; ix<nx; ++ix ) {
            velocity_x.clear();
            x = xvect[ix];
            for (unsigned int iy=0; iy<ny; ++iy) {
                velocity_y.clear();
                y = yvect[iy];
                for (unsigned int iz=0; iz<nz; ++iz) {
                    z = zvect[iz];
                    velocity_z = GetVelocity(x, y, z);
                    velocity_y.push_back(velocity_z);
                }
                velocity_x.push_back(velocity_y);
            }
            velocity.push_back(velocity_x);
        }
        return velocity;
    }

    double FrLinearWaveField::Fz(const double &z, const double &k) const {

        double result;

        if (m_infinite_depth) {
            result = exp(k * z);
        } else {
            result = cosh(k*(z+m_depth)) / sinh(k*m_depth);
        }

        return result;
    }

    double FrLinearWaveField::dFz(const double &z, const double &k) const {

        double result;

        if (m_infinite_depth) {
            result = k * exp(k * z);
        } else {
            result = k * sinh(k*(z+m_depth)) / sinh(k*m_depth);
        }

    }













    // REFACTORING --------------->>>>>>>>>>>>>>>>>>>>>











    // FrWaveField definitions

    FrWaveField_::FrWaveField_(FrFreeSurface_ *freeSurface) : m_freeSurface(freeSurface) {
        m_infinite_depth = freeSurface->GetOcean()->GetSeabed()->GetInfiniteDepth();
    }

    WAVE_MODEL FrWaveField_::GetWaveModel() const { return m_waveModel; }

    Velocity FrWaveField_::GetVelocity(double x, double y, double z, bool cutoff, FRAME_CONVENTION fc) const {

        if (cutoff) {
            auto wave_elevation = GetElevation(x, y, fc);
            if (wave_elevation < z) {
                return {0.,0.,0.};
            }
        }
        return GetVelocity(x, y, z, fc);
    }

    Velocity FrWaveField_::GetVelocity(const Position& worldPos, FRAME_CONVENTION fc) const {
        return GetVelocity(worldPos.GetX(), worldPos.GetY(), worldPos.GetZ(), fc);
    }

    Acceleration FrWaveField_::GetAcceleration(double x, double y, double z, bool cutoff, FRAME_CONVENTION fc) const {

        if (cutoff) {
            auto wave_elevation = GetElevation(x, y, fc);
            if (wave_elevation < z) {
                return {0.,0.,0.};
            }
        }
        return GetAcceleration(x, y, z, fc);
    }

    Acceleration FrWaveField_::GetAcceleration(const Position& worldPos, FRAME_CONVENTION fc) const {
        return GetAcceleration(worldPos.GetX(), worldPos.GetY(), worldPos.GetZ(), fc);
    }

    void FrWaveField_::Initialize() {
        m_infinite_depth = m_freeSurface->GetOcean()->GetSeabed()->GetInfiniteDepth();
        if (!m_infinite_depth) {c_depth = m_freeSurface->GetOcean()->GetDepth(NWU);};
    }

    void FrWaveField_::StepFinalize() {
        c_ramp = m_freeSurface->GetOcean()->GetEnvironment()->GetTimeRamp()->Get_y(c_time);
    }

    std::vector<std::vector<double>>
    FrWaveField_::GetElevation(const std::vector<double> &xVect, const std::vector<double> &yVect, FRAME_CONVENTION fc) const {
        auto nx = xVect.size();
        auto ny = yVect.size();

        std::vector<std::vector<double>> elevations;
        std::vector<double> elev;

        elevations.reserve(nx);
        elev.reserve(ny);

        double eta, x, y;
        for (unsigned int ix=0; ix<nx; ++ix) {
            elev.clear();
            x = xVect[ix];
            for (unsigned int iy=0; iy<ny; ++iy) {
                y = yVect[iy];
                eta = GetElevation(x, y, fc);
                elev.push_back(eta);
            }
            elevations.push_back(elev);
        }
        return elevations;
    }

    std::vector<std::vector<std::vector<Velocity>>>
    FrWaveField_::GetVelocity(const std::vector<double> &xvect, const std::vector<double> &yvect,
                              const std::vector<double> &zvect, FRAME_CONVENTION fc) const {
        auto nx = xvect.size();
        auto ny = yvect.size();
        auto nz = zvect.size();

        std::vector<std::vector<std::vector<Velocity>>> velocity;
        std::vector<std::vector<Velocity>> velocity_x;
        std::vector<Velocity> velocity_y;
        Velocity velocity_z;

        double x, y, z;

        for (unsigned int ix=0; ix<nx; ++ix ) {
            velocity_x.clear();
            x = xvect[ix];
            for (unsigned int iy=0; iy<ny; ++iy) {
                velocity_y.clear();
                y = yvect[iy];
                for (unsigned int iz=0; iz<nz; ++iz) {
                    z = zvect[iz];
                    velocity_z = GetVelocity(x, y, z, fc);
                    velocity_y.push_back(velocity_z);
                }
                velocity_x.push_back(velocity_y);
            }
            velocity.push_back(velocity_x);
        }
        return velocity;
    }

    void FrWaveField_::Update(double time) {
        c_time = time;
        if (!m_infinite_depth) {c_depth = m_freeSurface->GetOcean()->GetDepth(NWU);};
    }



    // FrNullWaveField definitions

    FrNullWaveField_::FrNullWaveField_(FrFreeSurface_* freeSurface) : FrWaveField_(freeSurface) {
        m_waveModel = NO_WAVES;
    }

    double FrNullWaveField_::GetElevation(double x, double y, FRAME_CONVENTION fc) const {
        return 0.;
    }

    Velocity FrNullWaveField_::GetVelocity(double x, double y, double z, FRAME_CONVENTION fc) const {
        return {0.,0.,0.};
    }

    Acceleration FrNullWaveField_::GetAcceleration(double x, double y, double z, FRAME_CONVENTION fc) const {
        return {0.,0.,0.};
    }

    std::vector<double> FrNullWaveField_::GetWaveFrequencies(FREQUENCY_UNIT unit) const {
        return std::vector<double>(1, 0.);
    }

    std::vector<double> FrNullWaveField_::GetWaveNumbers() const {
        return std::vector<double>(1, 0.);
    }

    std::vector<std::vector<double>> FrNullWaveField_::GetWaveAmplitudes() const {
        return std::vector<std::vector<double>>(1, std::vector<double>(1, 0.));
    }

    std::vector<double> FrNullWaveField_::GetWaveDirections(ANGLE_UNIT unit, FRAME_CONVENTION fc,
                                                            DIRECTION_CONVENTION dc) const {
        return std::vector<double>(1, 0.);
    }

    std::vector<std::vector<Complex>> FrNullWaveField_::GetComplexElevation(double x, double y, FRAME_CONVENTION fc) const {
        return std::vector<std::vector<Complex>>(1, std::vector<Complex>(1, 0.));
    }

//
//    // FrLinearWaveField definitions
//
//    FrLinearWaveField_::FrLinearWaveField_(FrFreeSurface_* freeSurface, LINEAR_WAVE_TYPE type) : FrWaveField_(freeSurface) {
//
//        SetType(type);
//        GenerateRandomWavePhases();
//        Initialize();
//        Update(0.);
//
//        m_waveRamp = std::make_shared<FrRamp>();
//        m_waveRamp->Initialize();
//
//        m_verticalFactor = std::make_unique<FrKinematicStretching_>();
//        m_verticalFactor->SetInfDepth(m_infinite_depth);
//
//    }
//
//    std::shared_ptr<FrLinearWaveProbe_> FrLinearWaveField_::NewWaveProbe(double x, double y) {
//        auto waveProbe = std::make_shared<FrLinearWaveProbe_>(this, x, y);
////        waveProbe->SetWaveField(this);
//        m_waveProbes.push_back(waveProbe);
//        return waveProbe;
//    }
//
////    void FrLinearWaveField_::AddWaveProbe(std::shared_ptr<FrLinearWaveProbe_> waveProbe) {
////        waveProbe->SetWaveField(this);
////        m_waveProbes.push_back(waveProbe);
////    }
//
//    std::shared_ptr<FrLinearWaveProbe_> FrLinearWaveField_::NewWaveProbe() {
//        auto waveProbe = std::make_shared<FrLinearWaveProbe_>(this);
////        waveProbe->SetWaveField(this);
//        m_waveProbes.push_back(waveProbe);
//        return waveProbe;
//    }
//
//    std::shared_ptr<FrLinearFlowSensor_> FrLinearWaveField_::NewFlowSensor(FrLinearWaveField_* waveField, double x, double y, double z) {
//        auto flowSensor = std::make_shared<FrLinearFlowSensor_>(this, x, y, z);
//        m_flowSensor.push_back(flowSensor);
//        return flowSensor;
//    }
//
//    std::vector<std::vector<double>> FrLinearWaveField_::_GetWaveAmplitudes() const {
//        std::vector<std::vector<double>> waveAmplitudes;
//        std::vector<double> ampl;
//        switch (m_linearWaveType) {
//
//            case LINEAR_REGULAR:
//                ampl.push_back(m_height * 0.5);
//                waveAmplitudes.push_back(ampl);
//                break;
//
//            case LINEAR_IRREGULAR:
//                ampl = m_waveSpectrum->GetWaveAmplitudes(m_nbFreq, m_minFreq, m_maxFreq);
//                waveAmplitudes.push_back(ampl);
//                break;
//
//            case LINEAR_DIRECTIONAL:
//                waveAmplitudes = m_waveSpectrum->GetWaveAmplitudes(m_nbFreq, m_minFreq, m_maxFreq,
//                                                                   m_nbDir, m_minDir, m_maxDir, m_meanDir);
//                break;
//        }
//        return waveAmplitudes;
//    }
//
//
//    void FrLinearWaveField_::SetStretching(FrStretchingType type) {
//
//        switch (type) {
//            case NO_STRETCHING:
//                m_verticalFactor = std::make_unique<FrKinematicStretching_>();
//                break;
//            case VERTICAL:
//                m_verticalFactor = std::make_unique<FrKinStretchingVertical_>();
//                break;
//            case EXTRAPOLATE:
//                m_verticalFactor = std::make_unique<FrKinStretchingExtrapol_>();
//                break;
//            case WHEELER:
//                m_verticalFactor = std::make_unique<FrKinStretchingWheeler_>(this);
//                break;
//            case CHAKRABARTI:
//                m_verticalFactor = std::make_unique<FrKinStretchingChakrabarti_>(this);
//            case DELTA:
//                m_verticalFactor = std::make_unique<FrKinStretchingDelta_>(this);
//            default:
//                m_verticalFactor = std::make_unique<FrKinematicStretching_>();
//                break;
//        }
//        m_verticalFactor->SetInfDepth(m_infinite_depth);
//    }
//
//    LINEAR_WAVE_TYPE FrLinearWaveField_::GetType() const {
//        return m_linearWaveType;
//    }
//
//    void FrLinearWaveField_::SetType(LINEAR_WAVE_TYPE type) {
//        m_linearWaveType = type;
//
//        switch (type) {
//            case LINEAR_REGULAR:
//                m_waveSpectrum = nullptr;
//                m_nbFreq = 1;
//                m_nbDir = 1;
//                m_minDir = m_meanDir;
//                m_maxDir = m_meanDir;
//                break;
//            case LINEAR_IRREGULAR:
//                m_nbDir = 1;
//                m_minDir = m_meanDir;
//                m_maxDir = m_meanDir;
//
//                // Creating a default waveSpectrum
//                SetWaveSpectrum(JONSWAP);
//                break;
//            case LINEAR_DIRECTIONAL:
//
//                break;
//        }
//    }
//
////    void FrLinearWaveField_::SetWaveHeight(double height) {
////        m_height = height;
////    }
////
////    void FrLinearWaveField_::SetWavePeriod(double period, FREQUENCY_UNIT unit) {
////        m_period = convert_frequency(period, unit, S);
////        Initialize();
////    }
//
//    unsigned int FrLinearWaveField_::GetNbFrequencies() const { return m_nbFreq; }
//
//    double FrLinearWaveField_::GetMinFrequency() const { return m_minFreq; }
//
//    double FrLinearWaveField_::GetMaxFrequency() const { return m_maxFreq; }
//
//    unsigned int FrLinearWaveField_::GetNbWaveDirections() const { return m_nbDir; }
//
//    double FrLinearWaveField_::GetMinWaveDirection() const { return m_minDir; }
//
//    double FrLinearWaveField_::GetMaxWaveDirection() const { return m_maxDir; }
//
//    double FrLinearWaveField_::GetMeanWaveDirection(ANGLE_UNIT unit) const {
//        double meanWaveDir = m_meanDir;
//        if (unit == DEG) {
//            meanWaveDir *= MU_180_PI;
//        }
//        return meanWaveDir;
//    }
//
//    void FrLinearWaveField_::SetMeanWaveDirection(const double meanDirection, ANGLE_UNIT unit) {
//        if (unit == DEG) {
//            m_meanDir = meanDirection * MU_PI_180;
//        } else {
//            m_meanDir = meanDirection;
//        }
//
//        if (!(m_linearWaveType == LINEAR_DIRECTIONAL)) {
//            m_minDir = m_meanDir;
//            m_maxDir = m_meanDir;
//        }
//    }
//
//    std::vector<double> FrLinearWaveField_::GetWaveDirections(ANGLE_UNIT unit) const {
//        std::vector<double> waveDirections;
//
//        if (m_linearWaveType != LINEAR_DIRECTIONAL) {
//            waveDirections.push_back(GetMeanWaveDirection(unit));
//
//        } else {
//            if (unit == DEG) {
//                waveDirections = linspace(m_minDir * MU_PI_180, m_maxDir * MU_PI_180, m_nbDir);
//            } else {
//                waveDirections = linspace(m_minDir, m_maxDir, m_nbDir);
//            }
//
//        }
//        return waveDirections;
//    }
//
//    void FrLinearWaveField_::SetWaveDirections(const double minDir, const double maxDir, const unsigned int nbDir,
//                                              ANGLE_UNIT unit) {
//
//        if (unit == DEG) {
//            m_minDir = minDir * MU_PI_180;
//            m_maxDir = maxDir * MU_PI_180;
//        } else {
//            m_minDir = minDir;
//            m_maxDir = maxDir;
//        }
//        m_nbDir = nbDir;
//
//        if (m_nbDir == 1) {
//            if (m_nbFreq == 1) {
//                SetType(LINEAR_REGULAR);
//            } else {
//                SetType(LINEAR_IRREGULAR);
//            }
//        }
//    }
//
//    std::vector<double> FrLinearWaveField_::GetWaveFrequencies(FREQUENCY_UNIT unit) const { return c_waveFrequencies; }
//
//    void FrLinearWaveField_::SetWavePulsations(const double minFreq, const double maxFreq, const unsigned int nbFreq,
//                                              FREQUENCY_UNIT unit) {
//
//        m_minFreq = convert_frequency(minFreq, unit, RADS);
//        m_maxFreq = convert_frequency(maxFreq, unit, RADS);
//        m_nbFreq = nbFreq;
//
//        if (nbFreq == 1) {
//            SetType(LINEAR_REGULAR);
//        }
//
//        Initialize();
//    }
//
//    void FrLinearWaveField_::Initialize() {
//        // TODO: aller chercher les infos dans FrOffshoreSystem
//
//        // Caching wave frequencies
//        c_waveFrequencies.clear();
//
//        if (m_linearWaveType == LINEAR_REGULAR) {
//            c_waveFrequencies.push_back(S2RADS(m_period));
//        } else {
//            c_waveFrequencies = linspace(m_minFreq, m_maxFreq, m_nbFreq);
//        }
//
//        // Caching wave numbers
//        c_waveNumbers.clear();
//
//        // waterHeight = environment->GetDepth()
//        // grav = environment->GetGrav()
//        // TODO: charger le hauteur d'eau et la gravite depuis
//        double waterHeight = 10.;
//        double grav = 9.81;
//        c_waveNumbers = SolveWaveDispersionRelation(waterHeight, c_waveFrequencies, grav);
//
//        if (m_wavePhases==nullptr) {
//            this->GenerateRandomWavePhases();
//        }
//
//        FrWaveField_::Initialize();
//        Update(0.);
//    }
//
//    std::vector<std::vector<double>>* FrLinearWaveField_::GetWavePhases() const {
//        return m_wavePhases.get();
//    }
//
//    void FrLinearWaveField_::SetWavePhases(std::vector<std::vector<double>> &wavePhases) {
//        assert(wavePhases.size() == m_nbDir);
//        for (auto& w: wavePhases) {
//            assert(w.size() == m_nbFreq);
//        }
//        m_wavePhases = std::make_unique<std::vector<std::vector<double>>>(wavePhases);
//    }
//
//    void FrLinearWaveField_::GenerateRandomWavePhases() {
//
//        m_wavePhases = std::make_unique<std::vector<std::vector<double>>>();
//
//        m_wavePhases->clear();
//        m_wavePhases->reserve(m_nbDir);
//
//        std::vector<double> phases;
//        phases.reserve(m_nbFreq);
//
//        if (m_linearWaveType == LINEAR_REGULAR) {
//            phases.push_back(0.);
//            m_wavePhases->push_back(phases);
//        } else {
//            std::random_device rd;
//            std::mt19937 gen(rd());
//            std::uniform_real_distribution<double> dis(0., MU_2PI);
//
//            for (uint idir=0; idir<m_nbDir; ++idir) {
//                phases.clear();
//                for (uint iw=0; iw<m_nbFreq; ++iw) {
//                    phases.push_back(dis(gen));
//                }
//                m_wavePhases->push_back(phases);
//            }
//        }
//    }
//
//    std::vector<double> FrLinearWaveField_::GetWaveLengths() const {
//        // TODO
//    }
//
//    std::vector<double> FrLinearWaveField_::GetWaveNumbers() const {
//        return c_waveNumbers;
//    }
//
//    std::vector<std::vector<std::complex<double>>>
//    FrLinearWaveField_::GetCmplxElevation(const double x, const double y, bool steady) const {
//
//
//        std::vector<std::vector<std::complex<double>>> cmplxElevations;  // is nbDir x nbFreq
//        cmplxElevations.reserve(m_nbDir);
//        std::vector<std::complex<double>> elev;
//        elev.reserve(m_nbFreq);
//
//        std::vector<double> w_;
//        w_.reserve(m_nbDir);
//        if (m_linearWaveType == LINEAR_DIRECTIONAL) {
//            auto wave_dirs = GetWaveDirections(RAD);
//            for (auto wave_dir: wave_dirs) {
//                w_.push_back(x * cos(wave_dir) + y * sin(wave_dir));
//            }
//        } else {
//            w_.push_back(x * cos(m_meanDir) + y * sin(m_meanDir));
//        }
//
//        std::vector<std::vector<double>> waveAmplitudes = _GetWaveAmplitudes();
//
//        std::complex<double> aik, val;
//        double ki, wi, wk_, phi_ik;
//
//        for (unsigned int idir=0; idir<m_nbDir; ++idir) {
//            elev.clear();
//            wk_ = w_[idir];
//
//            for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
//                aik = waveAmplitudes[idir][ifreq];
//                ki = c_waveNumbers[ifreq];  // FIXME: Ici, on doit avoir le nombre d'onde
//                phi_ik = m_wavePhases->at(idir)[ifreq];
//
//                val = aik * exp(JJ * (ki*wk_ + phi_ik));
//
//                if (!steady) {  // TODO : utiliser les valeurs mises en cache...
//                    val *= c_emjwt[ifreq];
//                }
//
//                elev.push_back(val);
//            }
//            cmplxElevations.push_back(elev);
//        }
//        return cmplxElevations;
//
//    }
//
//    std::vector<chrono::ChVector<std::complex<double>>>
//    FrLinearWaveField_::GetSteadyVelocity(const double x, const double y, const double z) const {
//
//        chrono::ChVector<std::complex<double>> vdiff;
//
//        // Initialization
//        std::vector<chrono::ChVector<std::complex<double>>> steadyVelocity;
//        steadyVelocity.reserve(m_nbFreq);
//        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
//            steadyVelocity.emplace_back(0.);
//        }
//
//        // Get the complex wave elevation
//        auto cmplxElevations = GetCmplxElevation(x, y, true);
//
//        // Wave direction
//        std::vector<double> angles;
//        angles.reserve(m_nbDir);
//        if (m_linearWaveType == LINEAR_DIRECTIONAL) {
//            angles = GetWaveDirections(RAD);
//        } else {
//            angles.push_back(m_meanDir);
//        }
//
//        // Pre-compute direction coefficient
//        std::vector<std::complex<double>> cxi, cyi;
//        for (auto& angle: angles) {
//            cxi.push_back(JJ*cos(angle));
//            cyi.push_back(JJ*sin(angle));
//        }
//
//        // Compute steady velocity
//        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
//
//            if (m_verticalFactor->IsSteady()) {
//                vdiff.x() = cos(m_meanDir) * JJ * c_waveNumbers[ifreq] * m_verticalFactor->Eval(x, y, z, c_waveNumbers[ifreq], m_depth);
//                vdiff.y() = sin(m_meanDir) * JJ * c_waveNumbers[ifreq] * m_verticalFactor->Eval(x, y, z, c_waveNumbers[ifreq], m_depth);
//                vdiff.z() = m_verticalFactor->EvalDZ(x, y, z, c_waveNumbers[ifreq], m_depth);
//            } else {
//                vdiff = chrono::ChVector<double>(1.);
//            }
//
//            vdiff *= c_waveFrequencies[ifreq] / c_waveNumbers[ifreq];
//
//            for (unsigned int idir=0; idir<angles.size(); ++idir) {
//
//                steadyVelocity[ifreq].x() += vdiff.x() * cmplxElevations[idir][ifreq];
//                steadyVelocity[ifreq].y() += vdiff.y() * cmplxElevations[idir][ifreq];
//                steadyVelocity[ifreq].z() += vdiff.z() * cmplxElevations[idir][ifreq];
//
//            }
//        }
//
//        return steadyVelocity;
//
//    }
//
//    std::vector<std::complex<double>> FrLinearWaveField_::GetSteadyElevation(const double x, const double y) const {
//        auto cmplxElevation = GetCmplxElevation(x, y, true);
//
//        std::vector<std::complex<double>> steadyElevation;
//        steadyElevation.reserve(m_nbFreq);
//        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
//            steadyElevation.emplace_back(0.);
//        }
//
//        // TODO: continuer
//        for (unsigned int idir=0; idir<m_nbDir; ++idir) {
//            for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
//                steadyElevation[ifreq] += cmplxElevation[idir][ifreq];
//            }
//        }
//        return steadyElevation;
//
//    }
//
//    void FrLinearWaveField_::SetWaveSpectrum(WAVE_SPECTRUM_TYPE type) {
//        m_waveSpectrum = MakeWaveSpectrum(type);
//    }
//
//    FrWaveSpectrum *FrLinearWaveField_::GetWaveSpectrum() const { return m_waveSpectrum.get(); }
//
////    void FrLinearWaveField_::SetReturnPeriod() {
////        // TODO
////    }
////
////    double FrLinearWaveField_::GetReturnPeriod() const {
////        // TODO
////    }
//
//    void FrLinearWaveField_::Update(double time) {
////        m_time = time;
//
//        // Updating exp(-jwt)
//        std::vector<double> w = GetWaveFrequencies(RADS);
//        c_emjwt.clear();
//        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
//            c_emjwt.push_back(exp(-JJ*w[ifreq]*time));
//        }
//
//    }
//
//    const std::vector<std::complex<double>> &FrLinearWaveField_::GetTimeCoeffs() const {
//        return c_emjwt;
//    }
//
//    const std::vector<std::complex<double>> &FrLinearWaveField_::GetTimeCoeffs(chrono::ChVector<double> vel) const {
//
//
//
//    }
//
//    std::vector<std::complex<double>> FrLinearWaveField_::GetTimeCoeffsDt() const {
//
//        std::vector<std::complex<double>> emjwtdt;
//        std::vector<double> w = GetWaveFrequencies(RADS);
//
//        emjwtdt.reserve(m_nbFreq);
//        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
//            emjwtdt.push_back( -JJ * w[ifreq] * c_emjwt[ifreq] );
//        }
//        return emjwtdt;
//    }
//
//    double FrLinearWaveField_::GetElevation(double x, double y) const {
//        // FIXME: appliquer la rampe ici aussi !!!
//        auto steadyElevation =  GetSteadyElevation(x, y);
//
//        double elevation = 0.;
//        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
//            elevation += std::imag( steadyElevation[ifreq] * c_emjwt[ifreq] );
//        }
//        return elevation;
//    }
//
//    Velocity FrLinearWaveField_::GetVelocity(double x, double y, double z) const {
//
//        auto steadyVelocity = GetSteadyVelocity(x, y, z);
//
//        std::vector<double> Kz(m_nbFreq, 1.), dKz(m_nbFreq, 1.);
//        if (!m_verticalFactor->IsSteady()) {
//            Kz = m_verticalFactor->Eval(x, y, z, c_waveNumbers, m_depth);
//            dKz = m_verticalFactor->EvalDZ(x, y, z, c_waveNumbers, m_depth);
//        }
//
//        Velocity velocity(0.,0.,0.);
//        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
//            velocity.x() += std::real( Kz[ifreq] * steadyVelocity[ifreq].x() * c_emjwt[ifreq]);
//            velocity.y() += std::real( Kz[ifreq] * steadyVelocity[ifreq].y() * c_emjwt[ifreq]);
//            velocity.z() += std::real( dKz[ifreq] * steadyVelocity[ifreq].z() * c_emjwt[ifreq]);
//        }
//        return velocity;
//    }
//
//    Acceleration FrLinearWaveField_::GetAcceleration(double x, double y, double z) const {
//
//        auto steadyVelocity = GetSteadyVelocity(x, y, z);
//        auto emjwt_dt = this->GetTimeCoeffsDt();
//
//        chrono::ChVector<std::complex<double>> acceleration(0.);
//        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
//            acceleration += steadyVelocity[ifreq] * emjwt_dt[ifreq];
//        }
//        Acceleration realAcceleration;
////        Acceleration realAcceleration = internal:: ChReal(acceleration);
////
////        if (m_waveRamp && m_waveRamp->IsActive()) {
////            m_waveRamp->Apply(m_time, realAcceleration);
////        }
//
//        return realAcceleration;
//    }
//
//    double FrLinearWaveField_::Fz(double z, double k) const {
//
//        double result;
//
//        if (m_infinite_depth) {
//            result = exp(k * z);
//        } else {
//            result = cosh(k*(z+m_depth)) / sinh(k*m_depth);
//        }
//
//        return result;
//    }
//
//    double FrLinearWaveField_::dFz(double z, double k) const {
//
//        double result;
//
//        if (m_infinite_depth) {
//            result = k * exp(k * z);
//        } else {
//            result = k * sinh(k*(z+m_depth)) / sinh(k*m_depth);
//        }
//
//    }


}  // end namespace frydom
