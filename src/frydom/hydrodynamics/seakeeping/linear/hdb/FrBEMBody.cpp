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

      for (unsigned int i = 0; i < 6; i++) { assert(mask(i) == 0 or mask(i) == 1); }

      for (unsigned int i = 0; i < 6; i++) {
        if (mask(i) == 1) {
          m_mask(i) = true;
          m_listDOF.push_back(i);
        } else {
          m_mask(i) = false;
        }
      }

      m_nbDOF = (unsigned int) mask.sum();

      m_matrix = Eigen::MatrixXd::Zero(6, m_nbDOF);
      unsigned int j = 0;
      for (unsigned int i = 0; i < 6; i++) {
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

    void FrWaveDriftPolarData::SetAngles(const std::vector<double> &angles) {
      m_table->SetX(angles);
    }

    void FrWaveDriftPolarData::SetFrequencies(const std::vector<double> &freqs) {
      m_table->SetY(freqs);
    }

    void FrWaveDriftPolarData::AddData(std::string &name, std::vector<double> coeffs) {
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

    template<typename OffshoreSystemType>
    unsigned int FrBEMBody<OffshoreSystemType>::GetNbFrequencies() const {
      return m_HDB->GetNbFrequencies();
    }

    template<typename OffshoreSystemType>
    std::vector<double> FrBEMBody<OffshoreSystemType>::GetFrequencies() const {
      return m_HDB->GetFrequencies();
    }

    template<typename OffshoreSystemType>
    unsigned int FrBEMBody<OffshoreSystemType>::GetNbWaveDirections() const {
      return m_HDB->GetNbWaveDirections();
    }

    template<typename OffshoreSystemType>
    std::vector<double>
    FrBEMBody<OffshoreSystemType>::GetWaveDirections(mathutils::ANGLE_UNIT angleUnit, FRAME_CONVENTION fc) const {
      return m_HDB->GetWaveDirections(angleUnit, fc);
    }

    template<typename OffshoreSystemType>
    unsigned int FrBEMBody<OffshoreSystemType>::GetNbBodies() const {
      return m_HDB->GetNbBodies();
    }

    template<typename OffshoreSystemType>
    unsigned int FrBEMBody<OffshoreSystemType>::GetNbTimeSamples() const {
      return m_HDB->GetNbTimeSamples();
    }

    template<typename OffshoreSystemType>
    void FrBEMBody<OffshoreSystemType>::Initialize() {

      /// This subroutine allocates the arrays for the hdb.

      assert(!m_forceModes.empty() && !m_motionModes.empty());

      auto nbForce = GetNbForceMode();

      /// --> Allocating arrays for excitations

      auto nbWaveDir = GetNbWaveDirections();
      m_excitationMask = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>(nbForce, nbWaveDir);
      m_excitationMask.setConstant(true);
      m_diffraction.reserve((unsigned long) nbWaveDir);
      m_froudeKrylov.reserve((unsigned long) nbWaveDir);
      m_excitation.reserve((unsigned long) nbWaveDir);

      auto nbFreq = GetNbFrequencies();
      for (int i = 0; i < nbWaveDir; ++i) {
        Eigen::MatrixXcd mat(nbForce, nbFreq);
        m_diffraction.push_back(mat);
        m_froudeKrylov.push_back(mat);
        m_excitation.push_back(mat);
      }

      /// --> Allocating arrays for radiation
      auto nbBodies = GetNbBodies();
      m_radiationMask.reserve(nbBodies);

      auto nbTime = GetNbTimeSamples();
      for (unsigned int ibody = 0; ibody < nbBodies; ++ibody) {

        auto body = m_HDB->GetBody(ibody);
        auto nbMotion = GetNbMotionMode();

        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> mask(nbForce, nbMotion);
        mask.setConstant(true);
        m_radiationMask.push_back(mask);

      }

    }

//    void FrBEMBody<OffshoreSystemType>::Finalize() {
//
//        // This function runs the interpolators.
//
//        BuildWaveExcitationInterpolators();
//    }

    //
    // Mask
    //
    template<typename OffshoreSystemType>
    void FrBEMBody<OffshoreSystemType>::SetForceMask(mathutils::Vector6d<int> mask) {
      m_forceMask.SetMask(mask);
    }

    template<typename OffshoreSystemType>
    void FrBEMBody<OffshoreSystemType>::SetMotionMask(mathutils::Vector6d<int> mask) {
      m_motionMask.SetMask(mask);
    }

    template<typename OffshoreSystemType>
    unsigned int FrBEMBody<OffshoreSystemType>::GetNbForceMode() const {
      return m_forceMask.GetNbMode();
    }

    template<typename OffshoreSystemType>
    unsigned int FrBEMBody<OffshoreSystemType>::GetNbMotionMode() const {
      return m_motionMask.GetNbMode();
    }

    //
    // Generalized modes
    //
    template<typename OffshoreSystemType>
    FrBEMForceMode *FrBEMBody<OffshoreSystemType>::GetForceMode(unsigned int imode) {
      assert(imode < 6);
      return &m_forceModes[imode];
    }

    template<typename OffshoreSystemType>
    FrBEMMotionMode *FrBEMBody<OffshoreSystemType>::GetMotionMode(unsigned int imode) {
      assert(imode < 6);
      return &m_motionModes[imode];
    }

    template<typename OffshoreSystemType>
    void FrBEMBody<OffshoreSystemType>::AddForceMode(FrBEMForceMode &mode) {
      m_forceModes.push_back(mode);
    }

    template<typename OffshoreSystemType>
    void FrBEMBody<OffshoreSystemType>::AddMotionMode(FrBEMMotionMode &mode) {
      m_motionModes.push_back(mode);
    }

    //
    // Setters
    //
    template<typename OffshoreSystemType>
    void FrBEMBody<OffshoreSystemType>::SetDiffraction(unsigned int iangle, const Eigen::MatrixXcd &diffractionMatrix) {
      assert(iangle < GetNbWaveDirections());
      assert(diffractionMatrix.rows() == 6);
      assert(diffractionMatrix.cols() == GetNbFrequencies());
      m_diffraction[iangle] = diffractionMatrix;
    }

    template<typename OffshoreSystemType>
    void
    FrBEMBody<OffshoreSystemType>::SetFroudeKrylov(unsigned int iangle, const Eigen::MatrixXcd &froudeKrylovMatrix) {
      assert(iangle < GetNbWaveDirections());
      assert(froudeKrylovMatrix.rows() == 6);
      assert(froudeKrylovMatrix.cols() == GetNbFrequencies());
      m_froudeKrylov[iangle] = froudeKrylovMatrix;
    }

    template<typename OffshoreSystemType>
    void FrBEMBody<OffshoreSystemType>::SetExcitation(unsigned int iangle, const Eigen::MatrixXcd &excitationMatrix) {
      assert(iangle < GetNbWaveDirections());
      assert(excitationMatrix.rows() == 6);
      assert(excitationMatrix.cols() == GetNbFrequencies());
      m_excitation[iangle] = excitationMatrix;
    }

    template<typename OffshoreSystemType>
    void FrBEMBody<OffshoreSystemType>::ComputeExcitation() {

      /// This subroutine computes the excitation loads from the diffraction loads and the Froude-Krylov loads.

      for (unsigned int iangle = 0; iangle < GetNbWaveDirections(); ++iangle) {
        m_excitation[iangle] = m_diffraction[iangle] + m_froudeKrylov[iangle];
      }
    }

    template<typename OffshoreSystemType>
    void FrBEMBody<OffshoreSystemType>::SetInfiniteAddedMass(FrBEMBody *BEMBodyMotion, const Eigen::MatrixXd &CMInf) {
      assert(CMInf.rows() == 6);
      assert(CMInf.cols() == 6);
      m_infiniteAddedMass[BEMBodyMotion] = CMInf;
    }

    template<typename OffshoreSystemType>
    void FrBEMBody<OffshoreSystemType>::SetImpulseResponseFunctionK(FrBEMBody *BEMBodyMotion,
                                                                    const std::vector<Eigen::MatrixXd> &listIRF) {

      for (auto &IRF: listIRF) {
        assert(IRF.rows() == 6);
        assert(IRF.cols() == GetNbTimeSamples());

        auto vtime = std::make_shared<std::vector<double>>(m_HDB->GetTimeDiscretization());

        auto vdata = std::make_shared<std::vector<mathutils::Vector6d<double>>>();
        for (unsigned int j = 0; j < IRF.cols(); ++j) {
          vdata->push_back(IRF.col(j));
        }

        auto interp = std::make_shared<mathutils::Interp1dLinear<double, mathutils::Vector6d<double>>>();
        interp->Initialize(vtime, vdata);

        m_interpK[BEMBodyMotion].push_back(interp);
      }
    }

    template<typename OffshoreSystemType>
    void
    FrBEMBody<OffshoreSystemType>::SetImpulseResponseFunctionKu(FrBEMBody *BEMBodyMotion,
                                                                const std::vector<Eigen::MatrixXd> &listIRF) {

      for (auto &IRF: listIRF) {
        assert(IRF.rows() == 6);
        assert(IRF.cols() == GetNbTimeSamples());

        auto vtime = std::make_shared<std::vector<double>>(m_HDB->GetTimeDiscretization());

        auto vdata = std::make_shared<std::vector<mathutils::Vector6d<double>>>();
        for (unsigned int j = 0; j < IRF.cols(); ++j) {
          vdata->push_back(IRF.col(j));
        }

        auto interp = std::make_shared<mathutils::Interp1dLinear<double, mathutils::Vector6d<double>>>();
        interp->Initialize(vtime, vdata);

        m_interpKu[BEMBodyMotion].push_back(interp);
      }
    }

    template<typename OffshoreSystemType>
    void
    FrBEMBody<OffshoreSystemType>::SetStiffnessMatrix(const mathutils::Matrix33<double> &hydrostaticStiffnessMatrix) {
      m_hydrostaticStiffnessMatrix = hydrostaticStiffnessMatrix;
    }

    template<typename OffshoreSystemType>
    void
    FrBEMBody<OffshoreSystemType>::SetStiffnessMatrix(const mathutils::Matrix66<double> &hydrostaticStiffnessMatrix) {
      m_hydrostaticStiffnessMatrix = hydrostaticStiffnessMatrix.block<3, 3>(2, 2);
    }

    template<typename OffshoreSystemType>
    void FrBEMBody<OffshoreSystemType>::SetWaveDrift() {
      m_waveDrift = std::make_unique<FrWaveDriftPolarData>();
    }

    //
    // Getters
    //
    template<typename OffshoreSystemType>
    Eigen::MatrixXcd FrBEMBody<OffshoreSystemType>::GetDiffraction(const unsigned int iangle) const {
      assert(iangle < this->GetNbWaveDirections());
      return m_diffraction[iangle];
    }

    template<typename OffshoreSystemType>
    Eigen::VectorXcd
    FrBEMBody<OffshoreSystemType>::GetDiffraction(const unsigned int iangle, const unsigned iforce) const {
      assert(iangle < this->GetNbWaveDirections());
      assert(iforce < 6);
      return m_diffraction[iangle].row(iforce);
    }

    template<typename OffshoreSystemType>
    Eigen::MatrixXcd FrBEMBody<OffshoreSystemType>::GetFroudeKrylov(const unsigned int iangle) const {
      assert(iangle < this->GetNbWaveDirections());
      return m_froudeKrylov[iangle];
    }

    template<typename OffshoreSystemType>
    Eigen::VectorXcd
    FrBEMBody<OffshoreSystemType>::GetFroudeKrylov(const unsigned int iangle, const unsigned iforce) const {
      assert(iangle < this->GetNbWaveDirections());
      assert(iforce < 6);
      return m_froudeKrylov[iangle].row(iforce);
    }

    template<typename OffshoreSystemType>
    Eigen::MatrixXcd FrBEMBody<OffshoreSystemType>::GetExcitation(const unsigned int iangle) const {
      assert(iangle < this->GetNbWaveDirections());
      return m_excitation[iangle];
    }

    template<typename OffshoreSystemType>
    Eigen::VectorXcd
    FrBEMBody<OffshoreSystemType>::GetExcitation(const unsigned int iangle, const unsigned iforce) const {
      assert(iangle < this->GetNbWaveDirections());
      assert(iforce < 6);
      return m_excitation[iangle].row(iforce);
    }

    template<typename OffshoreSystemType>
    mathutils::Matrix66<double> FrBEMBody<OffshoreSystemType>::GetInfiniteAddedMass(FrBEMBody *BEMBodyMotion) const {
      return m_infiniteAddedMass.at(BEMBodyMotion);
    }

    template<typename OffshoreSystemType>
    mathutils::Matrix66<double> FrBEMBody<OffshoreSystemType>::GetSelfInfiniteAddedMass() {
      return m_infiniteAddedMass[this];
    }

    template<typename OffshoreSystemType>
    mathutils::Interp1d<double, mathutils::Vector6d<double>> *
    FrBEMBody<OffshoreSystemType>::GetIRFInterpolatorK(FrBEMBody *BEMBodyMotion, unsigned int idof) {
      assert(idof < 6);
      return m_interpK[BEMBodyMotion][idof].get();
    };

    template<typename OffshoreSystemType>
    mathutils::Interp1d<double, mathutils::Vector6d<double>> *
    FrBEMBody<OffshoreSystemType>::GetIRFInterpolatorKu(FrBEMBody *BEMBodyMotion, unsigned int idof) {
      assert(idof < 6);
      return m_interpKu[BEMBodyMotion][idof].get();
    };

    template<typename OffshoreSystemType>
    std::shared_ptr<FrWaveDriftPolarData> FrBEMBody<OffshoreSystemType>::GetWaveDrift() const {
      return m_waveDrift;
    }


    //
    // Interpolators for the diffraction force
    //
    template<typename OffshoreSystemType>
    void FrBEMBody<OffshoreSystemType>::BuildDiffractionInterpolators() {

      // This subroutine interpolates the diffraction loads with respect to the wave direction.

      auto nbWaveDirections = GetNbWaveDirections();
      auto nbFreq = GetNbFrequencies();
      auto nbForceModes = GetNbForceMode();

      m_waveDirInterpolators.clear();
      m_waveDirInterpolators.reserve(nbForceModes);

      auto angles = std::make_shared<std::vector<double>>(GetWaveDirections(mathutils::RAD, NWU));

      auto interpolators = std::vector<mathutils::Interp1dLinear<double, std::complex<double>>>();
      interpolators.reserve(nbFreq);

      for (unsigned int imode = 0; imode < nbForceModes; ++imode) {

        interpolators.clear();

        for (unsigned int ifreq = 0; ifreq < nbFreq; ++ifreq) {

          auto coeffs = std::make_shared<std::vector<std::complex<double>>>();
          coeffs->reserve(nbWaveDirections);

          for (unsigned int iangle = 0; iangle < nbWaveDirections; ++iangle) {
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
