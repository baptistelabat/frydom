// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#include "FrBEMBody.h"

#include "FrHydroDB.h"



namespace frydom {

    //
    // FrDOFMask
    //

    void FrBEMDOFMask::SetMask(mathutils::Vector6d<int> mask) {

        for (unsigned int i=0; i<6; i++) { assert(mask(i) == 0 or mask(i) == 1); }

        for (unsigned int i=0; i<6; i++) {
            if (mask(i) == 1) {
                m_mask(i) = true;
                m_listDOF.push_back(i);
            } else {
                m_mask(i) = false;
            }
        }

        m_nbDOF = (unsigned int)mask.sum();

        m_matrix = Eigen::MatrixXd::Zero(6, m_nbDOF);
        unsigned int j=0;
        for (unsigned int i=0; i<6; i++) {
            if (mask(i)) {
                m_matrix(i, j) = 1.;
                j += 1;
            }
        }
    }

    mathutils::Vector6d<bool> FrBEMDOFMask::GetMask() const {
        return m_mask;
    }

    mathutils::MatrixMN<double> FrBEMDOFMask::GetMatrix() const {
        return m_matrix;
    }

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
        return m_table->Eval(name, x, y);
    }

    bool FrWaveDriftPolarData::HasSurge() const { return m_table->HasSerie("surge"); }

    bool FrWaveDriftPolarData::HasSway() const { return m_table->HasSerie("sway"); }

    bool FrWaveDriftPolarData::HasHeave() const { return m_table->HasSerie("heave"); }

    bool FrWaveDriftPolarData::HasPitch() const { return m_table->HasSerie("pitch"); }

    bool FrWaveDriftPolarData::HasRoll() const { return m_table->HasSerie("roll"); }

    bool FrWaveDriftPolarData::HasYaw() const { return m_table->HasSerie("yaw"); }
    //
    // FrBEMBody
    //

    unsigned int FrBEMBody::GetNbFrequencies() const {
        return m_HDB->GetNbFrequencies();
    }

    std::vector<double> FrBEMBody::GetFrequencies() const {
        return m_HDB->GetFrequencies();
    }

    unsigned int FrBEMBody::GetNbWaveDirections() const {
        return m_HDB->GetNbWaveDirections();
    }

    std::vector<double> FrBEMBody::GetWaveDirections(mathutils::ANGLE_UNIT angleUnit, FRAME_CONVENTION fc) const {
        return m_HDB->GetWaveDirections(angleUnit, fc);
    }

    unsigned int FrBEMBody::GetNbBodies() const {
        return m_HDB->GetNbBodies();
    }

    unsigned int FrBEMBody::GetNbTimeSamples() const {
        return m_HDB->GetNbTimeSamples();
    }

    void FrBEMBody::Initialize() {

        /// This subroutine allocates the arrays for the hdb.

        assert(!m_forceModes.empty() && !m_motionModes.empty());

        auto nbForce = GetNbForceMode();

        /// --> Allocating arrays for excitations

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

        /// --> Allocating arrays for radiation
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

//    void FrBEMBody::Finalize() {
//
//        // This function runs the interpolators.
//
//        BuildWaveExcitationInterpolators();
//    }

    //
    // Mask
    //

    void FrBEMBody::SetForceMask(mathutils::Vector6d<int> mask) {
        m_forceMask.SetMask(mask);
    }

    void FrBEMBody::SetMotionMask(mathutils::Vector6d<int> mask) {
        m_motionMask.SetMask(mask);
    }

    unsigned int FrBEMBody::GetNbForceMode() const {
        return m_forceMask.GetNbMode();
    }

    unsigned int FrBEMBody::GetNbMotionMode() const {
        return m_motionMask.GetNbMode();
    }

    //
    // Generalized modes
    //

    FrBEMForceMode* FrBEMBody::GetForceMode(unsigned int imode) {
        assert(imode < 6);
        return &m_forceModes[imode];
    }

    FrBEMMotionMode* FrBEMBody::GetMotionMode(unsigned int imode) {
        assert(imode < 6);
        return &m_motionModes[imode];
    }

    void FrBEMBody::AddForceMode(FrBEMForceMode& mode) {
        m_forceModes.push_back(mode);
    }

    void FrBEMBody::AddMotionMode(FrBEMMotionMode& mode) {
        m_motionModes.push_back(mode);
    }

    //
    // Setters
    //

    void FrBEMBody::SetDiffraction(unsigned int iangle, const Eigen::MatrixXcd &diffractionMatrix) {
        assert(iangle < GetNbWaveDirections());
        assert(diffractionMatrix.rows() == 6);
        assert(diffractionMatrix.cols() == GetNbFrequencies());
        m_diffraction[iangle] = diffractionMatrix;
    }

    void FrBEMBody::SetFroudeKrylov(unsigned int iangle, const Eigen::MatrixXcd& froudeKrylovMatrix) {
        assert(iangle < GetNbWaveDirections());
        assert(froudeKrylovMatrix.rows() == 6);
        assert(froudeKrylovMatrix.cols() == GetNbFrequencies());
        m_froudeKrylov[iangle] = froudeKrylovMatrix;
    }

    void FrBEMBody::SetExcitation(unsigned int iangle, const Eigen::MatrixXcd& excitationMatrix) {
        assert(iangle < GetNbWaveDirections());
        assert(excitationMatrix.rows() == 6);
        assert(excitationMatrix.cols() == GetNbFrequencies());
        m_excitation[iangle] = excitationMatrix;
    }

    void FrBEMBody::ComputeExcitation() {

        /// This subroutine computes the excitation loads from the diffraction loads and the Froude-Krylov loads.

        for (unsigned int iangle=0; iangle<GetNbWaveDirections(); ++iangle) {
            m_excitation[iangle] = m_diffraction[iangle] + m_froudeKrylov[iangle];
        }
    }

    void FrBEMBody::SetInfiniteAddedMass(FrBEMBody* BEMBodyMotion, const Eigen::MatrixXd& CMInf) {
        assert(CMInf.rows() == 6);
        assert(CMInf.cols() == 6);
        m_infiniteAddedMass[BEMBodyMotion] = CMInf;
    }

    void FrBEMBody::SetImpulseResponseFunctionK(FrBEMBody* BEMBodyMotion, const std::vector<Eigen::MatrixXd>& listIRF) {

        for (auto& IRF: listIRF) {
            assert(IRF.rows() == 6);
            assert(IRF.cols() == GetNbTimeSamples());

            auto vtime = std::make_shared<std::vector<double>>(m_HDB->GetTimeDiscretization());

            auto vdata = std::make_shared<std::vector<mathutils::Vector6d<double>>>();
            for (unsigned int j=0; j<IRF.cols(); ++j) {
                vdata->push_back(IRF.col(j));
            }

            auto interp = std::make_shared<mathutils::Interp1dLinear<double, mathutils::Vector6d<double>>>();
            interp->Initialize(vtime, vdata);

            m_interpK[BEMBodyMotion].push_back(interp);
        }
    }

    void FrBEMBody::SetImpulseResponseFunctionKu(FrBEMBody* BEMBodyMotion, const std::vector<Eigen::MatrixXd> &listIRF) {

        for (auto& IRF: listIRF) {
            assert(IRF.rows() == 6);
            assert(IRF.cols() == GetNbTimeSamples());

            auto vtime = std::make_shared<std::vector<double>>(m_HDB->GetTimeDiscretization());

            auto vdata = std::make_shared<std::vector<mathutils::Vector6d<double>>>();
            for (unsigned int j=0; j<IRF.cols(); ++j) {
                vdata->push_back(IRF.col(j));
            }

            auto interp = std::make_shared<mathutils::Interp1dLinear<double, mathutils::Vector6d<double>>>();
            interp->Initialize(vtime, vdata);

            m_interpKu[BEMBodyMotion].push_back(interp);
        }
    }

    void FrBEMBody::SetStiffnessMatrix(const mathutils::Matrix33<double>& hydrostaticStiffnessMatrix) {
        m_hydrostaticStiffnessMatrix = hydrostaticStiffnessMatrix;
    }

    void FrBEMBody::SetStiffnessMatrix(const mathutils::Matrix66<double>& hydrostaticStiffnessMatrix) {
        m_hydrostaticStiffnessMatrix = hydrostaticStiffnessMatrix.block<3, 3>(2, 2);
    }

    void FrBEMBody::SetWaveDrift() {
        m_waveDrift = std::make_unique<FrWaveDriftPolarData>();
    }

    //
    // Getters
    //

    Eigen::MatrixXcd FrBEMBody::GetDiffraction(const unsigned int iangle) const {
        assert(iangle < this->GetNbWaveDirections());
        return m_diffraction[iangle];
    }

    Eigen::VectorXcd FrBEMBody::GetDiffraction(const unsigned int iangle, const unsigned iforce) const {
        assert(iangle < this->GetNbWaveDirections());
        assert(iforce < 6);
        return m_diffraction[iangle].row(iforce);
    }

    Eigen::MatrixXcd FrBEMBody::GetFroudeKrylov(const unsigned int iangle) const {
        assert(iangle < this->GetNbWaveDirections());
        return m_froudeKrylov[iangle];
    }

    Eigen::VectorXcd FrBEMBody::GetFroudeKrylov(const unsigned int iangle, const unsigned iforce) const {
        assert(iangle < this->GetNbWaveDirections());
        assert(iforce < 6);
        return m_froudeKrylov[iangle].row(iforce);
    }

    Eigen::MatrixXcd FrBEMBody::GetExcitation(const unsigned int iangle) const {
        assert(iangle < this->GetNbWaveDirections());
        return m_excitation[iangle];
    }

    Eigen::VectorXcd FrBEMBody::GetExcitation(const unsigned int iangle, const unsigned iforce) const {
        assert(iangle < this->GetNbWaveDirections());
        assert(iforce < 6);
        return m_excitation[iangle].row(iforce);
    }

    mathutils::Matrix66<double> FrBEMBody::GetInfiniteAddedMass(FrBEMBody* BEMBodyMotion) const {
        return m_infiniteAddedMass.at(BEMBodyMotion);
    }

    mathutils::Matrix66<double> FrBEMBody::GetSelfInfiniteAddedMass() {
        return m_infiniteAddedMass[this];
    }

    mathutils::Interp1d<double, mathutils::Vector6d<double>>*
    FrBEMBody::GetIRFInterpolatorK(FrBEMBody* BEMBodyMotion, unsigned int idof) {
        assert(idof < 6);
        return m_interpK[BEMBodyMotion][idof].get();
    };

    mathutils::Interp1d<double, mathutils::Vector6d<double>>*
    FrBEMBody::GetIRFInterpolatorKu(FrBEMBody* BEMBodyMotion, unsigned int idof) {
        assert(idof < 6);
        return m_interpKu[BEMBodyMotion][idof].get();
    };

    std::shared_ptr<FrWaveDriftPolarData> FrBEMBody::GetWaveDrift() const {
        return m_waveDrift;
    }


    //
    // Interpolators for the diffraction force
    //

    void FrBEMBody::BuildDiffractionInterpolators() {

        // This subroutine interpolates the diffraction loads with respect to the wave direction.

        auto nbWaveDirections = GetNbWaveDirections();
        auto nbFreq = GetNbFrequencies();
        auto nbForceModes = GetNbForceMode();

        m_waveDirInterpolators.clear();
        m_waveDirInterpolators.reserve(nbForceModes);

        auto angles = std::make_shared<std::vector<double>>(GetWaveDirections(mathutils::RAD, NWU));

        auto interpolators = std::vector<mathutils::Interp1dLinear<double, std::complex<double>>>();
        interpolators.reserve(nbFreq);

        for (unsigned int imode = 0; imode<nbForceModes; ++imode) {

            interpolators.clear();

            for (unsigned int ifreq=0; ifreq<nbFreq; ++ifreq) {

                auto coeffs = std::make_shared<std::vector<std::complex<double>>>();
                coeffs->reserve(nbWaveDirections);

                for (unsigned int iangle=0; iangle<nbWaveDirections; ++iangle) {
                    auto data = GetDiffraction(iangle);
                    coeffs->push_back(data(imode, ifreq));
                }

                auto interpolator = mathutils::Interp1dLinear<double, std::complex<double>>();
                interpolator.Initialize(angles, coeffs);
                interpolators.push_back(interpolator);
            }
            m_waveDirInterpolators.push_back(interpolators);
        }
    }

}  // end namespace frydom
