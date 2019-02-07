//
// Created by frongere on 11/01/18.
//

#include "MathUtils/MathUtils.h"

#include "chrono/core/ChVectorDynamic.h"
#include "frydom/utils/FrEigen.h"
#include "FrHydroDB.h"
#include "FrBEMBody.h"
#include "frydom/core/junk/FrHydroBody.h"


using namespace mathutils;

namespace frydom {

    std::vector<double> FrDiscretization1D::GetVector() const {
        return linspace<double>(m_xmin, m_xmax, m_nx);
    }

    void FrBEMBody::Initialize() {
        assert(!m_ForceModes.empty() && !m_MotionModes.empty());

        auto nbForce = GetNbForceMode();

        // Allocating arrays for excitations
        auto NbWaveDir = m_HDB->GetNbWaveDirections();
        m_ExcitationMask = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>(nbForce, NbWaveDir);
        m_ExcitationMask.setConstant(true);
        m_Diffraction.reserve((unsigned long)NbWaveDir);
        m_FroudeKrylov.reserve((unsigned long)NbWaveDir);
        m_Excitation.reserve((unsigned long)NbWaveDir);

        auto NbFreq = m_HDB->GetNbFrequencies();
        for (int i=0; i<NbWaveDir; ++i) {
            Eigen::MatrixXcd mat(nbForce, NbFreq);
            m_Diffraction.push_back(mat);
            m_FroudeKrylov.push_back(mat);
            m_Excitation.push_back(mat);
        }

        // Allocating arrays for radiation
        auto NbBodies = m_HDB->GetNbBodies();
        m_RadiationMask.reserve(NbBodies);
        m_InfiniteAddedMass.reserve(NbBodies);
        m_AddedMass.reserve(NbBodies);
        m_RadiationDamping.reserve(NbBodies);
        m_ImpulseResponseFunction.reserve(NbBodies);

        auto NbTime = m_HDB->GetNbTimeSamples();
        for (unsigned int ibody=0; ibody<NbBodies; ++ibody) {

            auto body = m_HDB->GetBody(ibody);
            auto NbMotion = body->GetNbMotionMode();

            Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> mask(nbForce, NbMotion);
            mask.setConstant(true);
            m_RadiationMask.push_back(mask);

            Eigen::MatrixXd InfAddedMassMat(nbForce, NbMotion);
            m_InfiniteAddedMass.push_back(InfAddedMassMat);

            std::vector<Eigen::MatrixXd> added_mass_vector;
            added_mass_vector.reserve(NbMotion);  // TODO: voir si utile...
            std::vector<Eigen::MatrixXd> radiation_damping_vector;
            radiation_damping_vector.reserve(NbMotion);
            std::vector<Eigen::MatrixXd> impulse_response_fcn_vector;
            impulse_response_fcn_vector.reserve(NbMotion);

            for (unsigned int idof=0; idof<NbMotion; ++idof) {

                Eigen::MatrixXd mat(nbForce, NbFreq);
                added_mass_vector.push_back(mat);
                radiation_damping_vector.push_back(mat);

                Eigen::MatrixXd matTime(nbForce, NbTime);
                impulse_response_fcn_vector.push_back(matTime);
            }

            m_AddedMass.push_back(added_mass_vector);
            m_RadiationDamping.push_back(radiation_damping_vector);
            m_ImpulseResponseFunction.push_back(impulse_response_fcn_vector);
        }

    }

    void FrBEMBody::ComputeExcitation() {
        for (unsigned int iangle=0; iangle<m_HDB->GetNbWaveDirections(); ++iangle) {
            m_Excitation[iangle] = m_Diffraction[iangle] + m_FroudeKrylov[iangle];
        }
    }

    void FrBEMBody::SetDiffraction(const unsigned int iangle, const Eigen::MatrixXcd &diffractionMatrix) {
        assert(iangle < m_HDB->GetNbWaveDirections());
        assert(diffractionMatrix.rows() == GetNbForceMode());
        assert(diffractionMatrix.cols() == m_HDB->GetNbFrequencies());
        m_Diffraction[iangle] = diffractionMatrix;
    }

    void FrBEMBody::SetFroudeKrylov(const unsigned int iangle, const Eigen::MatrixXcd &froudeKrylovMatrix) {
        assert(iangle < m_HDB->GetNbWaveDirections());
        assert(froudeKrylovMatrix.rows() == GetNbForceMode());
        assert(froudeKrylovMatrix.cols() == m_HDB->GetNbFrequencies());
        m_FroudeKrylov[iangle] = froudeKrylovMatrix;
    }

    void FrBEMBody::SetInfiniteAddedMass(const unsigned int ibody, const Eigen::MatrixXd &CMInf) {
        assert(ibody < m_HDB->GetNbBodies());
        assert(CMInf.rows() == GetNbForceMode());
        assert(CMInf.cols() == m_HDB->GetBody(ibody)->GetNbMotionMode());
        m_InfiniteAddedMass[ibody] = CMInf;
    }

    void FrBEMBody::SetAddedMass(const unsigned int ibody, const unsigned int idof, const Eigen::MatrixXd &CM) {
        assert(ibody < m_HDB->GetNbBodies());
        assert(idof < m_HDB->GetBody(ibody)->GetNbMotionMode());
        assert(CM.rows() == GetNbForceMode());
        assert(CM.cols() == m_HDB->GetNbFrequencies());
        m_AddedMass[ibody][idof] = CM;
    }

    void FrBEMBody::SetRadiationDamping(const unsigned int ibody, const unsigned int idof, const Eigen::MatrixXd &CA) {
        assert(ibody < m_HDB->GetNbBodies());
        assert(idof < m_HDB->GetBody(ibody)->GetNbMotionMode());
        assert(CA.rows() == GetNbForceMode());
        assert(CA.cols() == m_HDB->GetNbFrequencies());
        m_RadiationDamping[ibody][idof] = CA;
    }

    void
    FrBEMBody::SetImpulseResponseFunction(const unsigned int ibody, const unsigned int idof, const Eigen::MatrixXd &IRF) {
        assert(ibody < m_HDB->GetNbBodies());
        assert(idof < m_HDB->GetBody(ibody)->GetNbMotionMode());
        assert(IRF.rows() == GetNbForceMode());
        assert(IRF.cols() == m_HDB->GetNbTimeSamples());
        m_ImpulseResponseFunction[ibody][idof] = IRF;
    }

    void FrBEMBody::SetSpeedDependentIRF(const unsigned int ibody, const unsigned int idof, const Eigen::MatrixXd &IRF) {
        assert(ibody < m_HDB->GetNbBodies());
        assert(idof < m_HDB->GetBody(ibody)->GetNbMotionMode());
        assert(IRF.rows() == GetNbForceMode());
        assert(IRF.cols() == m_HDB->GetNbTimeSamples());
        m_SpeedDependentIRF[ibody][idof] = IRF;
    }

    Eigen::MatrixXcd FrBEMBody::GetDiffraction(const unsigned int iangle) const {
        assert(iangle < m_HDB->GetNbWaveDirections());
        return m_Diffraction[iangle];
    }

    Eigen::VectorXcd FrBEMBody::GetDiffraction(const unsigned int iangle, const unsigned iforce) const {
        assert(iangle < m_HDB->GetNbWaveDirections());
        assert(iforce < GetNbForceMode());
        return m_Diffraction[iangle].row(iforce);
    }

    Eigen::MatrixXcd FrBEMBody::GetFroudeKrylov(const unsigned int iangle) const {
        assert(iangle < m_HDB->GetNbWaveDirections());
        return m_FroudeKrylov[iangle];
    }

    Eigen::VectorXcd FrBEMBody::GetFroudeKrylov(const unsigned int iangle, const unsigned iforce) const {
        assert(iangle < m_HDB->GetNbWaveDirections());
        assert(iforce < GetNbForceMode());
        return m_FroudeKrylov[iangle].row(iforce);
    }

    Eigen::MatrixXcd FrBEMBody::GetExcitation(const unsigned int iangle) const {
        assert(iangle < m_HDB->GetNbWaveDirections());
        return m_Excitation[iangle];
    }

    Eigen::VectorXcd FrBEMBody::GetExcitation(const unsigned int iangle, const unsigned iforce) const {
        assert(iangle < m_HDB->GetNbWaveDirections());
        assert(iforce < GetNbForceMode());
        return m_Excitation[iangle].row(iforce);
    }

    Eigen::MatrixXd FrBEMBody::GetInfiniteAddedMass(const unsigned int ibody) const {
        assert(ibody < m_HDB->GetNbBodies());
        return m_InfiniteAddedMass[ibody];
    }

    Eigen::MatrixXd FrBEMBody::GetSelfInfiniteAddedMass() const {
        return m_InfiniteAddedMass[m_ID];
    }

    Eigen::MatrixXd FrBEMBody::GetAddedMass(const unsigned int ibody, const unsigned int idof) const {
        assert(ibody < m_HDB->GetNbBodies());
        assert(idof < m_HDB->GetBody(ibody)->GetNbMotionMode());
        return m_AddedMass[ibody][idof];
    }

    Eigen::VectorXd
    FrBEMBody::GetAddedMass(const unsigned int ibody, const unsigned int idof, const unsigned int iforce) const {
        assert(ibody < m_HDB->GetNbBodies());
        assert(idof < m_HDB->GetBody(ibody)->GetNbMotionMode());
        assert(iforce < GetNbForceMode());
        return m_AddedMass[ibody][idof].row(iforce);
    }

    Eigen::MatrixXd FrBEMBody::GetSelfAddedMass(const unsigned int idof) const {
        assert(idof < GetNbMotionMode());
        return GetAddedMass(m_ID, idof);
    }

    Eigen::VectorXd FrBEMBody::GetSelfAddedMass(const unsigned int idof, const unsigned int iforce) const {
        assert(idof < GetNbMotionMode());
        assert(iforce < GetNbForceMode());
        return GetAddedMass(m_ID, idof, iforce);
    }

    Eigen::MatrixXd FrBEMBody::GetRadiationDamping(const unsigned int ibody, const unsigned int idof) const {
        assert(ibody < m_HDB->GetNbBodies());
        assert(idof < m_HDB->GetBody(ibody)->GetNbMotionMode());
        return m_RadiationDamping[ibody][idof];
    }

    Eigen::VectorXd
    FrBEMBody::GetRadiationDamping(const unsigned int ibody, const unsigned int idof, const unsigned int iforce) const {
        assert(ibody < m_HDB->GetNbBodies());
        assert(idof < m_HDB->GetBody(ibody)->GetNbMotionMode());
        assert(iforce < GetNbForceMode());
        return m_RadiationDamping[ibody][idof].row(iforce);
    }

    Eigen::MatrixXd FrBEMBody::GetSelfRadiationDamping(const unsigned int idof) const {
        assert(idof < GetNbMotionMode());
        return GetAddedMass(m_ID, idof);
    }

    Eigen::VectorXd FrBEMBody::GetselfRadiationDamping(const unsigned int idof, const unsigned int iforce) const {
        assert(idof < GetNbMotionMode());
        assert(iforce < GetNbForceMode());
        return GetAddedMass(m_ID, idof, iforce);
    }

    std::vector<Eigen::MatrixXd> FrBEMBody::GetImpulseResponseFunction(unsigned int ibody) const {
        assert(ibody < m_HDB->GetNbBodies());
        return m_ImpulseResponseFunction[ibody];
    }

    Eigen::MatrixXd FrBEMBody::GetImpulseResponseFunction(const unsigned int ibody, const unsigned int idof) const {
        assert(ibody < m_HDB->GetNbBodies());
        assert(idof < m_HDB->GetBody(ibody)->GetNbMotionMode());
        return m_ImpulseResponseFunction[ibody][idof];
    }

    Eigen::VectorXd FrBEMBody::GetImpulseResponseFunction(const unsigned int ibody, const unsigned int idof,
                                                          const unsigned int iforce) const {
        assert(ibody < m_HDB->GetNbBodies());
        assert(idof < m_HDB->GetBody(ibody)->GetNbMotionMode());
        assert(iforce < GetNbForceMode());
        return m_ImpulseResponseFunction[ibody][idof].row(iforce);
    }

    Eigen::MatrixXd FrBEMBody::GetSelfImpulseResponseFunction(const unsigned int idof) const {
        assert(idof < GetNbMotionMode());
        return GetAddedMass(m_ID, idof);
    }

    Eigen::VectorXd FrBEMBody::GetSelfImpulseResponseFunction(const unsigned int idof, const unsigned int iforce) const {
        assert(idof < GetNbMotionMode());
        assert(iforce < GetNbForceMode());
        return GetAddedMass(m_ID, idof, iforce);
    }

    std::vector<Eigen::MatrixXd> FrBEMBody::GetSpeedDependentIRF(unsigned int ibody) const {
        assert(ibody < m_HDB->GetNbBodies());
        return m_SpeedDependentIRF[ibody];
    }

    Eigen::MatrixXd FrBEMBody::GetSpeedDependentIRF(unsigned int ibody, unsigned int idof) const {
        assert(ibody < m_HDB->GetNbBodies());
        assert(idof < m_HDB->GetBody(ibody)->GetNbMotionMode());
        return m_SpeedDependentIRF[ibody][idof];
    }

    Eigen::VectorXd FrBEMBody::GetSpeedDependentIRF(unsigned int ibody, unsigned int idof, unsigned int iforce) const {
        assert(ibody < m_HDB->GetNbBodies());
        assert(idof < m_HDB->GetBody(ibody)->GetNbMotionMode());
        assert(iforce < GetNbForceMode());
        return m_SpeedDependentIRF[ibody][idof].row(iforce);
    }

    void FrBEMBody::FilterRadiation() {
        // TODO ...
//        for (unsigned int ibody=0; ibody<m_HDB->GetNbBodies(); ++ibody) {
//
//            auto CMinf = m_InfiniteAddedMass[ibody];
//            Eigen::Map<VectorXd> vect(CMinf.data(), CMinf.size());
//
//            vect = vect.cwiseAbs();
////            std::cout << vect << std::endl;
//            std::sort(vect.data(), vect.data()+vect.size());
//
//            std::cout << vect << "\n\n";
//            // TODO : terminer
//        }
    }

    void FrBEMBody::FilterExcitation() {
        // TODO
    }

    unsigned int FrBEMBody::GetNbFrequencies() const {
        return m_HDB->GetNbFrequencies();
    }

    std::vector<double> FrBEMBody::GetFrequencies() const {
        return m_HDB->GetFrequencies();
    }

    unsigned int FrBEMBody::GetNbWaveDirections() const {
        return m_HDB->GetNbWaveDirections();
    }

    std::vector<double> FrBEMBody::GetWaveDirections() const {
        return m_HDB->GetWaveDirections();
    }

    void FrBEMBody::BuildInterpolators() {
//        std::cout << "Generating HDB interpolators" << std::endl;
        // Wave Exxcitation interpolators
        BuildWaveExcitationInterpolators();
        // Radiation interpolators
//        BuildRadiationInterpolators(); // FIXME: c'est plutot a l'echelle de la HDB... ??? -> NON
    }

    void FrBEMBody::BuildWaveExcitationInterpolators() {

        auto nbWaveDirections = GetNbWaveDirections();
        auto nbFreq = GetNbFrequencies();
        auto nbForceModes = GetNbForceMode();

        m_waveDirInterpolators.clear();
        m_waveDirInterpolators.reserve(nbForceModes);

        auto angles = GetWaveDirections();
        auto angles_ptr = std::make_shared<std::vector<double>>();
        angles_ptr->reserve(nbWaveDirections);
        for (unsigned int iangle=0; iangle<nbWaveDirections; ++iangle) {
            angles_ptr->push_back(angles[iangle]);
        }

        Eigen::MatrixXcd data;
        auto interpolators = std::vector<Interp1dLinear<double, std::complex<double>>>();
        interpolators.reserve(nbFreq);

        for (unsigned int imode=0; imode<nbForceModes; ++imode) {

            interpolators.clear();

            for (unsigned int ifreq=0; ifreq<nbFreq; ++ifreq) {
                auto coeffs = std::make_shared<std::vector<std::complex<double>>>();
                coeffs->reserve(nbWaveDirections);

                for (unsigned int iangle=0; iangle<nbWaveDirections; ++iangle) {
                    data = GetExcitation(iangle);
//                    data = GetDiffraction(iangle);
                    coeffs->push_back(data(imode, ifreq));
                }

                auto interpolator = Interp1dLinear<double, std::complex<double>>();
                interpolator.Initialize(angles_ptr, coeffs);
                interpolators.push_back(interpolator);
            }
            m_waveDirInterpolators.push_back(interpolators);
        }
    }


    std::vector<Eigen::MatrixXcd>
    FrBEMBody::GetExcitationInterp(std::vector<double> waveFrequencies,
                                   std::vector<double> waveDirections,
                                   ANGLE_UNIT angleUnit) {  // TODO: utiliser angleUnit

        // Getting sizes
        auto nbFreqInterp = waveFrequencies.size();
        auto nbFreqBDD = GetNbFrequencies();
        auto nbDirInterp = waveDirections.size();
        auto nbForceMode = GetNbForceMode();

        std::vector<Eigen::MatrixXcd> Fexc;
        Fexc.reserve(nbDirInterp);

        // Building the database wave frequency vector as a shared vector
        auto freqsBDD = std::make_shared<std::vector<double>>();
        freqsBDD->reserve(nbFreqBDD);
        auto omega = GetFrequencies();
        for (unsigned int ifreq=0; ifreq<nbFreqBDD; ++ifreq) {
            freqsBDD->push_back(omega[ifreq]);
        }

        // shared vector to hold database frequency coefficients
        auto freqCoeffs = std::make_shared<std::vector<std::complex<double>>>();
        freqCoeffs->reserve(nbFreqBDD);

        for (auto direction: waveDirections) {

            auto FexcDir = Eigen::MatrixXcd(nbForceMode, nbFreqInterp);
            for (unsigned int imode=0; imode<nbForceMode; ++imode) {

                // Building a frequency interpolator for mode imode and requested wave direction
                freqCoeffs->clear();
                for (unsigned int ifreq=0; ifreq<nbFreqBDD; ++ifreq) {
                    freqCoeffs->push_back(m_waveDirInterpolators[imode][ifreq](direction));
                }
                auto freqInterpolator = Interp1dLinear<double, std::complex<double>>();
                freqInterpolator.Initialize(freqsBDD, freqCoeffs); // TODO: ajouter une methode clear() afin de ne pas instancier l'interpolateur a chaque iteration (sortir l'instanciation des boucles...)

                auto freqCoeffsInterp = freqInterpolator(waveFrequencies); // TODO: sortir l'instanciation des boucles...
                for (unsigned int ifreq=0; ifreq<nbFreqInterp; ++ifreq) {
                    FexcDir(imode, ifreq) = freqCoeffsInterp[ifreq];
                }

            }
            Fexc.push_back(FexcDir);
        }

        return Fexc;

    }

    std::vector<std::vector<double>>
    FrBEMBody::GetEncounterFrequencies(std::vector<double> waveFrequencies,
                                       std::vector<double> waveDirections,
                                       std::vector<double> waveNumbers,
                                       chrono::ChVector<double> frame_velocity,
                                       ANGLE_UNIT angleUnit) {

        std::vector<std::vector<double>> waveEncounterFrequencies;
        std::vector<double> waveEncounterFrequencies_freq;


        auto nbFreq = waveFrequencies.size();
        auto nbDir = waveDirections.size();

        // Velocity component in wave direction
        auto angle = Normalize_0_2PI(atan2(frame_velocity.y(), frame_velocity.x()));
        auto norm_speed = frame_velocity.Length();

        std::vector<double> velocity;
        velocity.reserve(nbDir);
        for (unsigned int idir=0; idir<nbDir; ++idir) {
            velocity.push_back(norm_speed * cos(waveDirections[idir]*DEG2RAD - angle));
        }

        // Encounter frequencies

        waveEncounterFrequencies.reserve(nbDir);
        for (unsigned int idir=0; idir<nbDir; idir++) {
            waveEncounterFrequencies_freq.clear();
            waveEncounterFrequencies_freq.reserve(nbFreq);
            for (unsigned int ifreq=0; ifreq<nbFreq; ifreq++) {
                waveEncounterFrequencies_freq.push_back(waveFrequencies[ifreq] - waveNumbers[ifreq] * velocity[idir]);
            }
            waveEncounterFrequencies.push_back(waveEncounterFrequencies_freq);
        }

        return waveEncounterFrequencies;
    }

    std::vector<Eigen::MatrixXcd>
    FrBEMBody::GetExcitationInterp(std::vector<double> waveFrequencies,
                                   std::vector<double> waveDirections,
                                   std::vector<double> waveNumbers,
                                   chrono::ChVector<double> frame_velocity,
                                   ANGLE_UNIT angleUnit) {

        auto nbFreqInterp = waveFrequencies.size();
        auto nbFreqBDD = GetNbFrequencies();
        auto nbDirInterp = waveDirections.size();
        auto nbForceMode = GetNbForceMode();

        std::vector<Eigen::MatrixXcd> Fexc;
        Fexc.reserve(nbDirInterp);

        // Building the database wave frequency vector as a shared vector
        auto freqsBDD = std::make_shared<std::vector<double>>();
        freqsBDD->reserve(nbFreqBDD);
        auto omega = GetFrequencies();
        for (unsigned int ifreq=0; ifreq<nbFreqBDD; ++ifreq) {
            freqsBDD->push_back(omega[ifreq]);
        }

        // shared vector to hold database frequency coefficients
        auto freqCoeffs = std::make_shared<std::vector<std::complex<double>>>();
        freqCoeffs->reserve(nbFreqBDD);

        auto waveEncounterFrequencies = GetEncounterFrequencies(waveFrequencies,
                                        waveDirections, waveNumbers, frame_velocity,
                                        angleUnit);


        double direction;
        for (unsigned int idir=0; idir<nbDirInterp; idir++) {

            direction = waveDirections[idir];

            auto FexcDir = Eigen::MatrixXcd(nbForceMode, nbFreqInterp);
            for (unsigned int imode=0; imode<nbForceMode; ++imode) {

                // Building a frequency interpolator for mode imode and requested wave direction
                freqCoeffs->clear();
                for (unsigned int ifreq=0; ifreq<nbFreqBDD; ++ifreq) {
                    freqCoeffs->push_back(m_waveDirInterpolators[imode][ifreq](direction));
                }
                auto freqInterpolator = Interp1dLinear<double, std::complex<double>>();
                freqInterpolator.Initialize(freqsBDD, freqCoeffs); // TODO: ajouter une methode clear() afin de ne pas instancier l'interpolateur a chaque iteration (sortir l'instanciation des boucles...)

                auto freqCoeffsInterp = freqInterpolator(waveEncounterFrequencies[idir]); // TODO: sortir l'instanciation des boucles...

                for (unsigned int ifreq=0; ifreq<nbFreqInterp; ++ifreq) {
                    FexcDir(imode, ifreq) = freqCoeffsInterp[ifreq];
                }

            }
            Fexc.push_back(FexcDir);

        }

        return Fexc;

    }

    void FrBEMBody::GenerateImpulseResponseFunctions() {

        // Frequencies
        auto wmin = m_HDB->GetMinFrequency();
        auto wmax = m_HDB->GetMaxFrequency();
        auto nbFreq = m_HDB->GetNbFrequencies();
        auto omega = m_HDB->GetFrequencies();
        auto dw = m_HDB->GetStepFrequency();

        // Time information for Impulse response function
        auto tf = m_HDB->GetFinalTime();
        auto dt = m_HDB->GetTimeStep();

        if (dt == 0.) {
            throw std::runtime_error("Time discretization for impulse response functions has not been "
                                             "initialized in the hydrodynamic database");
        }

//        auto dt = p_dt;
//        if (dt == 0.) {
//            // Ensuring a time sample satisfying largely the shannon theorem (5x by security...)
//            dt = MU_2PI / (5. * wmax);
//        }

        auto time = arange<double>(0, tf, dt);

        auto nbTime = time.size();

        m_ImpulseResponseFunction.clear();
        m_ImpulseResponseFunction.swap(m_ImpulseResponseFunction);
        m_ImpulseResponseFunction.reserve(m_HDB->GetNbBodies());

        unsigned int nbMotion, nbForce;
        std::vector<double> integrand;
        integrand.reserve(nbFreq);
        double val;

        // Initializing the 1d integrator
//        auto myIntegrator = Integrate1d<double>();
//        myIntegrator.SetIntegrationMethod(TRAPEZOIDAL);
//        myIntegrator.SetXmin(wmin);
//        myIntegrator.SetXmax(wmax);
//        myIntegrator.SetNbPoints(nbFreq);


        for (unsigned int iBody=0; iBody<m_HDB->GetNbBodies(); iBody++) {

            nbMotion = m_HDB->GetBody(iBody)->GetNbMotionMode();
            nbForce = m_HDB->GetBody(iBody)->GetNbForceMode();

            std::vector<Eigen::MatrixXd> body_i_impulseResponseFunctions;
            body_i_impulseResponseFunctions.reserve(nbMotion);

            for (unsigned int iMotion=0; iMotion<nbMotion; iMotion++) {

                Eigen::MatrixXd localIRF(nbForce, nbTime);

                for (unsigned int iForce=0; iForce<nbForce; iForce++) {
                    auto localPotentialDamping = m_RadiationDamping[iBody][iMotion].row(iForce);

                    // Performing integrations
                    for (unsigned int iTime=0; iTime<nbTime; iTime++) {
                        integrand.clear();
                        for (unsigned int iFreq=0; iFreq<nbFreq; iFreq++) {
                            val = localPotentialDamping[iFreq] * cos(omega[iFreq] * time[iTime]);
                            integrand.push_back(val);
                        }

                        // Integration
//                        myIntegrator.SetY(integrand);
                        localIRF(iForce, iTime) = Trapz(integrand, dw);
                    }
                }
                localIRF /= MU_PI_2;
                body_i_impulseResponseFunctions.push_back(localIRF);
            }
            m_ImpulseResponseFunction.push_back(body_i_impulseResponseFunctions);
        }  // Loop on bodies


        // ##CC : write impulse response function into output file
        /**
        std::string filename = "ImpulseResponseFunction.dat";
        std::fstream myfile;
        myfile.open(filename, std::ios::ate | std::ios::app);

        myfile << "#time";
        for (unsigned int iMotion=0; iMotion<nbMotion; iMotion++) {
            for (unsigned int iForce=0; iForce<nbForce; iForce++) {
                myfile << ";K" << std::to_string(iMotion) << std::to_string(iForce);
            }
        }
        myfile << std::endl;

        for (unsigned int itime=0; itime<nbTime; itime++) {
            myfile << time[itime];
            for (unsigned int iMotion=0; iMotion<nbMotion; iMotion++) {
                for (unsigned int iForce=0; iForce<nbForce; iForce++) {
                    myfile << ";" << m_ImpulseResponseFunction[0][iMotion](iForce,itime);
                }
            }
            myfile << std::endl;
        }
        myfile.close();
        **/
        // ##CC

    }

    void FrBEMBody::GenerateSpeedDependentIRF() {

        auto wmin = m_HDB->GetMinFrequency();
        auto wmax = m_HDB->GetMaxFrequency();
        auto nbFreq = m_HDB->GetNbFrequencies();
        auto omega = m_HDB->GetFrequencies();
        auto dw = m_HDB->GetStepFrequency();

        auto tf = m_HDB->GetFinalTime();
        auto dt = m_HDB->GetTimeStep();

        if (std::abs(dt) < DBL_EPSILON) {
            throw std::runtime_error("Time discretization for impulse response functions has not been "
                                             "initialized in the hydrodynamic database");
        }

        auto time = arange<double>(0, tf, dt);

        auto nbTime = time.size();

        m_SpeedDependentIRF.clear();
        m_SpeedDependentIRF.swap(m_SpeedDependentIRF);
        m_SpeedDependentIRF.reserve(m_HDB->GetNbBodies());

        unsigned int nbMotion, nbForce;
        std::vector<double> integrand;
        integrand.reserve(nbFreq);
        double val;

        for (unsigned int iBody=0; iBody < m_HDB->GetNbBodies(); iBody++) {

            nbMotion = m_HDB->GetBody(iBody)->GetNbMotionMode();
            nbForce = m_HDB->GetBody(iBody)->GetNbForceMode();

            std::vector<Eigen::MatrixXd> body_i_IRF;
            body_i_IRF.reserve(nbMotion);

            // iMotion : 0 -> 3
            Eigen::MatrixXd localIRF(nbForce, nbTime);
            localIRF.setZero();
            for (unsigned int iMotion=0; iMotion<4; iMotion++) {
                body_i_IRF.push_back(localIRF);
            }

            // iMotion : 4
            for (unsigned int iForce=0; iForce<nbForce; iForce++) {
                double Ainf = m_InfiniteAddedMass[iBody](2,iForce);
                auto kernel = m_AddedMass[iBody][2].row(iForce);
                for (unsigned int iTime=0; iTime<nbTime; iTime++) {
                    integrand.clear();
                    for (unsigned int iFreq=0; iFreq<nbFreq; iFreq++) {
                        val = (kernel[iFreq] - Ainf) * cos(omega[iFreq] * time[iTime]);
                        integrand.push_back(val);
                    }
                    localIRF(iForce, iTime) = Trapz(integrand, dw);
                }
            }
            localIRF /= MU_PI_2;
            body_i_IRF.push_back(localIRF);

            localIRF.setZero();

            //iMotion : 5
            for (unsigned int iForce=0; iForce<nbForce; iForce++) {
                double Ainf = m_InfiniteAddedMass[iBody](1,iForce);
                auto kernel = m_AddedMass[iBody][1].row(iForce);
                for (unsigned int iTime=0; iTime<nbTime; iTime++) {
                    integrand.clear();
                    for (unsigned int iFreq=0; iFreq<nbFreq; iFreq++) {
                        val = (Ainf - kernel[iFreq]) * cos(omega[iFreq] * time[iTime]);
                        integrand.push_back(val);
                    }
                    localIRF(iForce, iTime) = Trapz(integrand, dw);
                }
            }
            localIRF /= MU_PI_2;
            body_i_IRF.push_back(localIRF);

            m_SpeedDependentIRF.push_back(body_i_IRF);
        }

        // ##CC : write impulse response function into output file
        /**
        std::string filename = "SpeedDependentIRF.dat";
        std::fstream myfile;
        myfile.open(filename, std::ios::ate | std::ios::app);

        myfile << "#time";
        for (unsigned int iMotion=0; iMotion<nbMotion; iMotion++) {
            for (unsigned int iForce=0; iForce<nbForce; iForce++) {
                myfile << ";Ku" << std::to_string(iMotion) << std::to_string(iForce);
            }
        }
        myfile << std::endl;

        for (unsigned int itime=0; itime<nbTime; itime++) {
            myfile << time[itime];
            for (unsigned int iMotion=0; iMotion<nbMotion; iMotion++) {
                for (unsigned int iForce=0; iForce<nbForce; iForce++) {
                    myfile << ";" << m_SpeedDependentIRF[0][iMotion](iForce,itime);
                }
            }
            myfile << std::endl;
        }
        myfile.close();
        **/
        // ##CC
    }

    void FrBEMBody::IntLoadResidual_Mv(const unsigned int off,
                                       chrono::ChVectorDynamic<>& R,
                                       const chrono::ChVectorDynamic<>& w,
                                       const double c) {

        Eigen::VectorXd q(6);
        for (int i=0; i<6; i++) { q(i) = w(off+i);}

        Eigen::VectorXd Mv = c * m_InfiniteAddedMass[0] * q;
        auto Mw = chrono::ChVector<>(Mv(0), Mv(1), Mv(2));
        auto Iw = chrono::ChVector<>(Mv(3), Mv(4), Mv(5));
        R.PasteSumVector(Mw, off, 0);
        R.PasteSumVector(Iw, off+3, 0);
    }

    void FrBEMBody::SetBEMVariables() {
        m_hydroBody->SetVariables(variablesHydro);
        m_hydroBody->GetVariables<FrVariablesBEMBodyMass>()->SetInfiniteAddedMass(m_InfiniteAddedMass[0]);
    }































    /// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< REFACTORING


    //
    // FrWaveDriftPolarCoeff
    //

    FrWaveDriftPolarData::FrWaveDriftPolarData() {
        m_table = std::make_unique<mathutils::LookupTable2d<>>();
    }

    void FrWaveDriftPolarData::SetAngles(const std::vector<double>& angles) {
        m_table->SetX(angles);
    }

    void FrWaveDriftPolarData::SetFrequencies(const std::vector<double>& freqs) {
        m_table->SetY(freqs);
    }

    void FrWaveDriftPolarData::AddData(std::string& name, std::vector<double> coeffs) {
        m_table->AddData(name, coeffs);
    }

    double FrWaveDriftPolarData::Eval(const std::string name, double x, double y) const {
        m_table->Eval(name, x, y);
    }
    //
    // FrBEMBody
    //

    unsigned int FrBEMBody_::GetNbFrequencies() const {
        return m_HDB->GetNbFrequencies();
    }

    std::vector<double> FrBEMBody_::GetFrequencies() const {
        return m_HDB->GetFrequencies();
    }

    unsigned int FrBEMBody_::GetNbWaveDirections() const {
        return m_HDB->GetNbWaveDirections();
    }

    std::vector<double> FrBEMBody_::GetWaveDirections() const {
        return m_HDB->GetWaveDirections();
    }

    unsigned int FrBEMBody_::GetNbBodies() const {
        return m_HDB->GetNbBodies();
    }

    unsigned int FrBEMBody_::GetNbTimeSamples() const {
        return m_HDB->GetNbTimeSamples();
    }


    void FrBEMBody_::Initialize() {

        assert(!m_forceModes.empty() && !m_motionModes.empty());

        auto nbForce = GetNbForceMode();

        // --> Allocating arrays for excitations

        auto nbWaveDir = GetNbWaveDirections();
        m_excitationMask = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>(nbForce, nbWaveDir);
        m_excitationMask.setConstant(true);
        m_diffraction.reserve((unsigned long)nbWaveDir);
        m_froudeKrylov.reserve((unsigned long)nbWaveDir);
        m_excitation.reserve((unsigned long)nbWaveDir);

        auto nbFreq = GetNbFrequencies();
        for (int i=0; i<nbWaveDir; ++i) {
            Eigen::MatrixXcd mat(nbForce, nbFreq);
            m_diffraction.push_back(mat);
            m_froudeKrylov.push_back(mat);
            m_excitation.push_back(mat);
        }

        // --> Allocating arrays for radiation

        auto nbBodies = GetNbBodies();
        m_radiationMask.reserve(nbBodies);

        auto nbTime = GetNbTimeSamples();
        for (unsigned int ibody=0; ibody<nbBodies; ++ibody) {

            auto body = m_HDB->GetBody(ibody);
            auto nbMotion = GetNbMotionMode();

            Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> mask(nbForce, nbMotion);
            mask.setConstant(true);
            m_radiationMask.push_back(mask);

        }

    }

    void FrBEMBody_::Finalize() {
        BuildWaveExcitationInterpolators();
        //BuildIRFInterpolators();
    }

    //
    // Generalized modes
    //

    FrBEMForceMode_* FrBEMBody_::GetForceMode(unsigned int imode) {
        assert(imode < GetNbForceMode());
        return &m_forceModes[imode];
    }

    FrBEMMotionMode_* FrBEMBody_::GetMotionMode(unsigned int imode) {
        assert(imode < GetNbMotionMode());
        return &m_motionModes[imode];
    }

    void FrBEMBody_::AddForceMode(FrBEMForceMode_& mode) {
        m_forceModes.push_back(mode);
    }

    void FrBEMBody_::AddMotionMode(FrBEMMotionMode_& mode) {
        m_motionModes.push_back(mode);
    }

    //
    // Setters
    //

    void FrBEMBody_::SetDiffraction(unsigned int iangle, const Eigen::MatrixXcd &diffractionMatrix) {
        assert(iangle < GetNbWaveDirections());
        assert(diffractionMatrix.rows() == GetNbForceMode());
        assert(diffractionMatrix.cols() == GetNbFrequencies());
        m_diffraction[iangle] = diffractionMatrix;
    }

    void FrBEMBody_::SetFroudeKrylov(unsigned int iangle, const Eigen::MatrixXcd& froudeKrylovMatrix) {
        assert(iangle < GetNbWaveDirections());
        assert(froudeKrylovMatrix.rows() == GetNbForceMode());
        assert(froudeKrylovMatrix.cols() == GetNbFrequencies());
        m_froudeKrylov[iangle] = froudeKrylovMatrix;
    }

    void FrBEMBody_::SetExcitation(unsigned int iangle, const Eigen::MatrixXcd& excitationMatrix) {
        assert(iangle < GetNbWaveDirections());
        assert(excitationMatrix.rows() == GetNbForceMode());
        assert(excitationMatrix.cols() == GetNbFrequencies());
        m_excitation[iangle] = excitationMatrix;
    }

    void FrBEMBody_::ComputeExcitation() {
        for (unsigned int iangle=0; iangle<GetNbWaveDirections(); ++iangle) {
            m_excitation[iangle] = m_diffraction[iangle] + m_froudeKrylov[iangle];
        }
    }

    void FrBEMBody_::SetInfiniteAddedMass(FrBEMBody_* BEMBodyMotion, const Eigen::MatrixXd& CMInf) {
        assert(CMInf.rows() == 6);
        assert(CMInf.cols() == 6);
        m_infiniteAddedMass[BEMBodyMotion] = CMInf;
    }

    void FrBEMBody_::SetImpulseResponseFunctionK(FrBEMBody_* BEMBodyMotion, const std::vector<Eigen::MatrixXd>& listIRF) {

        for (auto& IRF: listIRF) {
            assert(IRF.rows() == 6);
            assert(IRF.cols() == GetNbTimeSamples());

            auto vtime = std::make_shared<std::vector<double>>(m_HDB->GetTimeDiscretization());

            auto vdata = std::make_shared<std::vector<Vector6d<double>>>();
            for (unsigned int j=0; j<IRF.cols(); ++j) {
                vdata->push_back(IRF.col(j));
            }

            auto interp = std::make_shared<Interp1dLinear<double, Vector6d<double>>>();
            interp->Initialize(vtime, vdata);

            m_interpK[BEMBodyMotion].push_back(interp);
        }
    }

    void FrBEMBody_::SetImpulseResponseFunctionKu(FrBEMBody_* BEMBodyMotion, const std::vector<Eigen::MatrixXd> &listIRF) {

        for (auto& IRF: listIRF) {
            assert(IRF.rows() == 6);
            assert(IRF.cols() == GetNbTimeSamples());

            auto vtime = std::make_shared<std::vector<double>>(m_HDB->GetTimeDiscretization());

            auto vdata = std::make_shared<std::vector<Vector6d<double>>>();
            for (unsigned int j=0; j<IRF.cols(); ++j) {
                vdata->push_back(IRF.col(j));
            }

            auto interp = std::make_shared<Interp1dLinear<double, Vector6d<double>>>();
            interp->Initialize(vtime, vdata);

            m_interpKu[BEMBodyMotion].push_back(interp);
        }
    }

    void FrBEMBody_::SetStiffnessMatrix(const Matrix33& hydrostaticStiffnessMatrix) {
        m_hydrostaticStiffnessMatrix = hydrostaticStiffnessMatrix;
    }

    void FrBEMBody_::SetStiffnessMatrix(const Matrix66<double>& hydrostaticStiffnessMatrix) {
        m_hydrostaticStiffnessMatrix = hydrostaticStiffnessMatrix.block<3, 3>(2, 2);
    }

    void FrBEMBody_::SetWaveDrift() {
        m_waveDrift = std::make_unique<FrWaveDriftPolarData>();
    }

    //
    // Getters
    //

    Eigen::MatrixXcd FrBEMBody_::GetDiffraction(const unsigned int iangle) const {
        assert(iangle < this->GetNbWaveDirections());
        return m_diffraction[iangle];
    }

    Eigen::VectorXcd FrBEMBody_::GetDiffraction(const unsigned int iangle, const unsigned iforce) const {
        assert(iangle < this->GetNbWaveDirections());
        assert(iforce < this->GetNbForceMode());
        return m_diffraction[iangle].row(iforce);
    }

    Eigen::MatrixXcd FrBEMBody_::GetFroudeKrylov(const unsigned int iangle) const {
        assert(iangle < this->GetNbWaveDirections());
        return m_froudeKrylov[iangle];
    }

    Eigen::VectorXcd FrBEMBody_::GetFroudeKrylov(const unsigned int iangle, const unsigned iforce) const {
        assert(iangle < this->GetNbWaveDirections());
        assert(iforce < this->GetNbForceMode());
        return m_froudeKrylov[iangle].row(iforce);
    }

    Eigen::MatrixXcd FrBEMBody_::GetExcitation(const unsigned int iangle) const {
        assert(iangle < this->GetNbWaveDirections());
        return m_excitation[iangle];
    }

    Eigen::VectorXcd FrBEMBody_::GetExcitation(const unsigned int iangle, const unsigned iforce) const {
        assert(iangle < this->GetNbWaveDirections());
        assert(iforce < this->GetNbForceMode());
        return m_excitation[iangle].row(iforce);
    }

    mathutils::Matrix66<double> FrBEMBody_::GetInfiniteAddedMass(FrBEMBody_* BEMBodyMotion) const {
        return m_infiniteAddedMass.at(BEMBodyMotion);
    }

    mathutils::Matrix66<double> FrBEMBody_::GetSelfInfiniteAddedMass() {
        return m_infiniteAddedMass[this];
    }

    Interp1d<double, Vector6d<double>>* FrBEMBody_::GetIRFInterpolatorK(FrBEMBody_* BEMBodyMotion, unsigned int idof) {
        assert(idof < 6);
        return m_interpK[BEMBodyMotion][idof].get();
    };

    Interp1d<double, Vector6d<double>>* FrBEMBody_::GetIRFInterpolatorKu(FrBEMBody_* BEMBodyMotion, unsigned int idof) {
        assert(idof < 6);
        return m_interpKu[BEMBodyMotion][idof].get();
    };

    //
    // Interpolators for the excitation force
    //

    void FrBEMBody_::BuildWaveExcitationInterpolators() {

        auto nbWaveDirections = GetNbWaveDirections();
        auto nbFreq = GetNbFrequencies();
        auto nbForceModes = GetNbForceMode();

        m_waveDirInterpolators.clear();
        m_waveDirInterpolators.reserve(nbForceModes);

        auto angles = std::make_shared<std::vector<double>>(GetWaveDirections());

        auto interpolators = std::vector<Interp1dLinear<double, std::complex<double>>>();
        interpolators.reserve(nbFreq);

        for (unsigned int imode =0; imode<nbForceModes; ++imode) {

            interpolators.clear();

            for (unsigned int ifreq=0; ifreq<nbFreq; ++ifreq) {

                auto coeffs = std::make_shared<std::vector<std::complex<double>>>();
                coeffs->reserve(nbWaveDirections);

                for (unsigned int iangle=0; iangle<nbWaveDirections; ++iangle) {
                    auto data = GetExcitation(iangle);
                    coeffs->push_back(data(imode, ifreq));
                }

                auto interpolator = Interp1dLinear<double, std::complex<double>>();
                interpolator.Initialize(angles, coeffs);
                interpolators.push_back(interpolator);
            }
            m_waveDirInterpolators.push_back(interpolators);
        }
    }

    std::vector<Eigen::MatrixXcd>
    FrBEMBody_::GetExcitationInterp(std::vector<double> waveFrequencies,
                                    std::vector<double> waveDirections,
                                    ANGLE_UNIT angleUnit) {

        // --> Getting sizes

        auto nbFreqInterp = waveFrequencies.size();
        auto nbFreqBDD = GetNbFrequencies();
        auto nbDirInterp = waveDirections.size();
        auto nbForceMode = GetNbForceMode();

        std::vector<Eigen::MatrixXcd> Fexc;
        Fexc.reserve(nbDirInterp);

        // -> Building interpolator and return vector

        auto freqsBDD = std::make_shared<std::vector<double>>(GetFrequencies());

        auto freqCoeffs = std::make_shared<std::vector<std::complex<double>>>();
        freqCoeffs->reserve(nbFreqBDD);

        for (auto direction: waveDirections) {

            auto excitationForceDir = Eigen::MatrixXcd(nbForceMode, nbFreqInterp);

            for (unsigned int imode=0; imode<nbForceMode; ++imode) {

                freqCoeffs->clear();
                for (unsigned int ifreq=0; ifreq<nbFreqBDD; ++ifreq) {
                    freqCoeffs->push_back(m_waveDirInterpolators[imode][ifreq](direction));
                }

                auto freqInterpolator = Interp1dLinear<double, std::complex<double>>();
                freqInterpolator.Initialize(freqsBDD, freqCoeffs);

                auto freqCoeffsInterp = freqInterpolator(waveFrequencies);
                for (unsigned int ifreq=0; ifreq<nbFreqInterp; ++ifreq) {
                    excitationForceDir(imode, ifreq) = freqCoeffsInterp[ifreq];
                }
            }
            Fexc.push_back(excitationForceDir);
        }
        return Fexc;
    }

    std::shared_ptr<FrWaveDriftPolarData> FrBEMBody_::GetWaveDrift() const {
        return m_waveDrift;
    }

}  // end namespace frydom
