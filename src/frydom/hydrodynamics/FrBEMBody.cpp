//
// Created by frongere on 11/01/18.
//

#include "MathUtils.h"

#include "FrHydroDB.h"
#include "FrBEMBody.h"
//#include "frydom/core/FrHydroBody.h"


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

    }

}  // end namespace frydom