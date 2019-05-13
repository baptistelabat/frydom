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


#include "FrHydroDB.h"

#include "FrHydroMapper.h"
#include "FrBEMBody.h"
#include "frydom/IO/FrHDF5.h"

#include "MathUtils/VectorGeneration.h"
#include "MathUtils/Constants.h"


// TODO: utiliser boost/multi_array.hpp a la place des vector<vector<Eigen::Matrix>>> ?????


namespace frydom {

    // ----------------------------------------------
    // FrDiscretization1D
    // ----------------------------------------------

    std::vector<double> FrDiscretization1D::GetVector() const {
        return mathutils::linspace<double>(m_xmin, m_xmax, m_nx);
    }

    void FrDiscretization1D::SetStep(double delta) {
        m_nx = 1 + (unsigned int)((m_xmax - m_xmin) / delta);
    }

    double FrDiscretization1D::GetStep() const {
        return (m_xmax-m_xmin) / double(m_nx-1);
    }

    // ----------------------------------------------------
    // FrHydroDB
    // ----------------------------------------------------

    void FrHydroDB::SetWaveDirectionDiscretization(const double minAngle, const double maxAngle,
                                                    const unsigned int nbAngle) {
        m_waveDirectionDiscretization.SetMin(minAngle);
        m_waveDirectionDiscretization.SetMax(maxAngle);
        m_waveDirectionDiscretization.SetNbSample(nbAngle);
    }

    void FrHydroDB::SetTimeDiscretization(const double finalTime, const unsigned int nbTimeSamples) {
        m_timeDiscretization.SetMin(0.);
        m_timeDiscretization.SetMax(finalTime);
        m_timeDiscretization.SetNbSample(nbTimeSamples);
    }

    void FrHydroDB::SetFrequencyDiscretization(const double minFreq, const double maxFreq,
                                                const unsigned int nbFreq) {
        m_frequencyDiscretization.SetMin(minFreq);
        m_frequencyDiscretization.SetMax(maxFreq);
        m_frequencyDiscretization.SetNbSample(nbFreq);
    }

    std::vector<double> FrHydroDB::GetWaveDirections(mathutils::ANGLE_UNIT angleUnit, FRAME_CONVENTION fc) const {

        auto waveDir = m_waveDirectionDiscretization.GetVector();

        if (angleUnit == mathutils::RAD) {
            for (auto& angle: waveDir) { angle *= DEG2RAD; }
        }

        if (IsNED(fc)) {
            for (auto& angle: waveDir) { angle = -angle; }
        }

        return waveDir;
    }

    FrBEMBody* FrHydroDB::NewBody(std::string bodyName) {
        m_bodies.push_back( std::make_unique<FrBEMBody>(GetNbBodies(), bodyName, this));
        return m_bodies.back().get();
    }

    FrBEMBody* FrHydroDB::GetBody(std::shared_ptr<FrBody> body) {
        return m_mapper->GetBEMBody(body.get());
    }

    FrBEMBody* FrHydroDB::GetBody(FrBody* body) {
        return m_mapper->GetBEMBody(body);
    }

    FrBEMBody* FrHydroDB::GetBody(int ibody) {
        return m_bodies[ibody].get();
    }

    FrBody* FrHydroDB::GetBody(FrBEMBody* body) {
        return m_mapper->GetBody(body);
    }

    FrHydroMapper* FrHydroDB::GetMapper() {
        return m_mapper.get();
    }

    void FrHydroDB::Map(FrBEMBody* BEMBody, FrBody* body, std::shared_ptr<FrEquilibriumFrame> eqFrame) {
        m_mapper->Map(BEMBody, body, eqFrame);
    }

    void FrHydroDB::Map(int iBEMBody, FrBody* body, std::shared_ptr<FrEquilibriumFrame> eqFrame) {
        m_mapper->Map(m_bodies[iBEMBody].get(), body, eqFrame);
    }

    std::unordered_map<FrBEMBody*, FrBody*>::iterator FrHydroDB::begin() {
        return m_mapper->begin();
    };

    std::unordered_map<FrBEMBody*, FrBody*>::iterator FrHydroDB::end() {
        return m_mapper->end();
    };

    FrHydroDB::FrHydroDB(std::string h5file) {

        // Constructor of the class.
        // This function generates the FrHydroDB_ object from the *.HDB5 input file.

        // Object for reading *.HDB5 input file.
        FrHDF5Reader reader;
        reader.SetFilename(h5file);

        // Reading environmental data and the number of bodies.
        m_gravityAcc = reader.ReadDouble("/GravityAcc");
        m_waterDensity = reader.ReadDouble("/WaterDensity");
        m_normalizationLength = reader.ReadDouble("/NormalizationLength");
        m_waterDepth = reader.ReadDouble("/WaterDepth");
        m_nbody = reader.ReadInt("/NbBody");

        // Creation of the mapper object for hydrodynamic bodies.
        m_mapper = std::make_unique<FrHydroMapper>();

        // ----> Reading discretization path
        std::string discretization_path = "/Discretizations";

        // -----------> Reading frequency path
        std::string frequency_discretization_path = discretization_path + "/Frequency";
        auto NbFreq = reader.ReadInt(frequency_discretization_path + "/NbFrequencies"); // Number of frequencies.
        auto MinFreq = reader.ReadDouble(frequency_discretization_path + "/MinFrequency"); // Minimum frequency.
        auto MaxFreq = reader.ReadDouble(frequency_discretization_path + "/MaxFrequency"); // Maximum frequency.
        this->SetFrequencyDiscretization(MinFreq, MaxFreq, (uint)NbFreq); // Settings the frequency discretization.

        // ------------> Reading waves directions
        std::string wave_direction_discretization_path = discretization_path + "/WaveDirections";
        auto NbWaveDir = reader.ReadInt(wave_direction_discretization_path + "/NbWaveDirections"); // Number of wave directions.
        auto MinWaveDir = reader.ReadDouble(wave_direction_discretization_path + "/MinAngle"); // Minimum wave direction.
        auto MaxWaveDir = reader.ReadDouble(wave_direction_discretization_path + "/MaxAngle"); // Maximum wave direction.
        this->SetWaveDirectionDiscretization(MinWaveDir, MaxWaveDir, (uint)NbWaveDir); // Settings the wave direction discretization.

        // -------------> Reading time discretization
        std::string time_discretization_path = discretization_path + "/Time";
        auto NbTimeSample = reader.ReadInt(time_discretization_path + "/NbTimeSample"); // Number of time steps.
        auto FinalTime = reader.ReadDouble(time_discretization_path + "/FinalTime"); // Duration.
        this->SetTimeDiscretization(FinalTime, (uint)NbTimeSample); // Settings the time discretization.

        // -------------> Reading data from Body
        char buffer[20];
        std::string bodyPath = "/Bodies/Body_";

        // Loop over all the bodies subject to hydrodynamic loads.
        for (unsigned int ibody=0; ibody<m_nbody; ++ibody) {

            sprintf(buffer, "%d", ibody);
            auto ibodyPath = bodyPath + buffer;
            auto bodyName = reader.ReadString(ibodyPath + "/BodyName");

            // Creation of a new FrBEMBody.
            auto BEMBody = this->NewBody(bodyName);

            // Initialization from the *.HDB5 input file.
            Position bodyPosition = reader.ReadDoubleArray(ibodyPath + "/BodyPosition");
            BEMBody->SetPosition(bodyPosition);

            auto ID = reader.ReadInt(ibodyPath + "/ID");
            assert(BEMBody->GetID() == ID);

            // Reading the modes of a body.
            this->ModeReader(reader, ibodyPath, BEMBody);

            this->MaskReader(reader, ibodyPath, BEMBody);

            // Allocation of the arrays for the hdb.
            BEMBody->Initialize();

        }

        // -----------> Reading the hydrodynamic coefficients
        for (unsigned int ibody=0; ibody<m_nbody; ++ibody) {

            sprintf(buffer, "%d", ibody);
            auto ibodyPath = bodyPath + buffer;

            auto BEMBody = this->GetBody(ibody);

            // Reading of the excitation loads.
            this->ExcitationReader(reader, ibodyPath, BEMBody);

            // Reading of the radiation coefficients (infinite added mass, impulse response functions).
            this->RadiationReader(reader, ibodyPath, BEMBody);

            // Reading the wave drift coefficients.
            if (reader.GroupExist(ibodyPath + "/WaveDrift")) {
                this->WaveDriftReader(reader, ibodyPath + "/WaveDrift", BEMBody);
            }

            // Reading the hydrostatic matrix.
            if (reader.GroupExist(ibodyPath + "/Hydrostatic")) {
                this->HydrostaticReader(reader, ibodyPath + "/Hydrostatic", BEMBody);
            }

        }
    }

    void FrHydroDB::MaskReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody) {
        auto maskPath = path + "/Mask";
        BEMBody->SetForceMask(reader.ReadIntArray(maskPath + "/ForceMask"));
        BEMBody->SetMotionMask(reader.ReadIntArray(maskPath + "/MotionMask"));
    }

    void FrHydroDB::ModeReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody) {

        // This subroutine reads the modes of a body.

        char buffer[20];

        // ----- Force Mode ----------------------

        auto nbForceModes = reader.ReadInt(path + "/Modes/NbForceModes");

        auto modePath = path + "/Modes/ForceModes/Mode_";

        for (unsigned int iforce=0; iforce<nbForceModes; ++iforce) {

            sprintf(buffer, "%d", iforce);
            auto imodePath = modePath + buffer;

            // Building the force mode
            FrBEMForceMode mode;

            auto modeType = reader.ReadString(imodePath + "/Type");
            Direction direction = reader.ReadDoubleArray(imodePath + "/Direction");
            mode.SetDirection(direction);

            if (modeType == "ANGULAR") {
                Position point = reader.ReadDoubleArray(imodePath + "/Point");
                mode.SetTypeANGULAR();
                mode.SetPointPosition(point);
            } else {
                mode.SetTypeLINEAR();
            }

            // Adding the mode to the BEMBody
            BEMBody->AddForceMode(mode);

        }

        // --------------- Motion Mode --------------------

        auto nbMotionModes = reader.ReadInt(path + "/Modes/NbMotionModes");

        modePath = path + "/Modes/MotionModes/Mode_";

        for (unsigned int imotion=0; imotion<nbForceModes; ++imotion) {
            sprintf(buffer, "%d", imotion);
            auto imodePath = modePath + buffer;

            FrBEMMotionMode mode;

            auto modeType = reader.ReadString(imodePath + "/Type");
            Direction direction = reader.ReadDoubleArray(imodePath + "/Direction");

            mode.SetDirection(direction);

            if (modeType == "ANGULAR") {
                Position point = reader.ReadDoubleArray(imodePath + "/Point");
                mode.SetTypeANGULAR();
                mode.SetPointPosition(point);
            } else {
                mode.SetTypeLINEAR();
            }

            // Adding the mode to the BEMBody
            BEMBody->AddMotionMode(mode);
        }

    }

    void FrHydroDB::ExcitationReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody) {

        // This subroutine reads the excitation loads from the *.HDB5 input file.

        char buffer[20];

        auto diffractionPath = path + "/Excitation/Diffraction";
        auto froudeKrylovPath = path + "/Excitation/FroudeKrylov/";

        auto forceMaskMatrix = BEMBody->GetForceMaskMatrix();

        for (unsigned int iwaveDir=0; iwaveDir < GetNbWaveDirections(); ++iwaveDir) {

            sprintf(buffer, "/Angle_%d", iwaveDir);

            // -> Diffraction loads.
            auto diffractionWaveDirPath = diffractionPath + buffer;
            auto diffractionRealCoeffs = reader.ReadDoubleArray(diffractionWaveDirPath + "/RealCoeffs");
            auto diffractionImagCoeffs = reader.ReadDoubleArray(diffractionWaveDirPath + "/ImagCoeffs");

            Eigen::MatrixXcd diffractionCoeffs;
            diffractionCoeffs = diffractionRealCoeffs + MU_JJ * diffractionImagCoeffs; // In complex.
            BEMBody->SetDiffraction(iwaveDir, forceMaskMatrix * diffractionCoeffs);

            // -> Froude-Krylov loads.
            auto froudeKrylovWaveDirPath = froudeKrylovPath + buffer;
            auto froudeKrylovRealCoeffs = reader.ReadDoubleArray(froudeKrylovWaveDirPath + "/RealCoeffs");
            auto froudeKrylovImagCoeffs = reader.ReadDoubleArray(froudeKrylovWaveDirPath + "/ImagCoeffs");

            Eigen::MatrixXcd froudeKrylovCoeffs;
            froudeKrylovCoeffs = froudeKrylovRealCoeffs + MU_JJ * froudeKrylovImagCoeffs; // In complex.
            BEMBody->SetFroudeKrylov(iwaveDir, forceMaskMatrix * froudeKrylovCoeffs);

        }

        // Computation of the excitation loads the diffraction and Froude-Krylov loads.
        BEMBody->ComputeExcitation();
    }

    void FrHydroDB::RadiationReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody) {

        // This subroutine reads the radiation coefficients from the *.HDB5 input file.

        char buffer[20];

        auto radiationPath = path + "/Radiation";

        auto forceMaskMatrix = BEMBody->GetForceMaskMatrix();

        auto nbTimeSamples = GetNbTimeSamples();

        for (unsigned int ibodyMotion=0; ibodyMotion < m_nbody; ++ibodyMotion) {

            sprintf(buffer, "/BodyMotion_%d", ibodyMotion);

            auto bodyMotion = this->GetBody(ibodyMotion);

            auto motionMaskMatrix = bodyMotion->GetMotionMaskMatrix();

            // Reading the infinite added mass matrix for the body.
            auto infiniteAddedMassPath = radiationPath + buffer + "/InfiniteAddedMass";
            auto infiniteAddedMassHDB = reader.ReadDoubleArray(infiniteAddedMassPath);
            auto infiniteAddedMass = forceMaskMatrix * infiniteAddedMassHDB * motionMaskMatrix.transpose();
            BEMBody->SetInfiniteAddedMass(bodyMotion, infiniteAddedMass);

            // Reading the impulse response functions.
            auto IRFPath = radiationPath + buffer + "/ImpulseResponseFunctionK";

            std::vector<Eigen::MatrixXd> impulseResponseFunctionsK;

            unsigned int idof = 0;
            for (unsigned int imotion=0; imotion<6; ++imotion) {
                if (bodyMotion->GetMotionMask(imotion)) {
                    sprintf(buffer, "/DOF_%d", idof);
                    auto mat = reader.ReadDoubleArray(IRFPath + buffer);
                    impulseResponseFunctionsK.push_back(forceMaskMatrix * mat);
                    idof += 1;
                } else {
                    impulseResponseFunctionsK.push_back(Eigen::MatrixXd::Zero(6, nbTimeSamples));
                }
            }
            BEMBody->SetImpulseResponseFunctionK(bodyMotion, impulseResponseFunctionsK);

            // Reading the impulse response functions in case of forward speed.
            sprintf(buffer, "/BodyMotion_%d", ibodyMotion);
            auto IRFUPath = radiationPath + buffer + "/ImpulseResponseFunctionKU";

            if (reader.GroupExist(IRFUPath)) {

                std::vector<Eigen::MatrixXd> impulseResponseFunctionsKU;

                for (unsigned int imotion=0; imotion<6; ++imotion) {

                    if(bodyMotion->GetMotionMask(imotion)) {
                        sprintf(buffer, "/DOF_%d", idof);
                        auto mat = reader.ReadDoubleArray(IRFUPath + buffer);
                        impulseResponseFunctionsKU.push_back(forceMaskMatrix * mat);
                        idof += 1;
                    } else {
                        impulseResponseFunctionsKU.push_back(Eigen::MatrixXd::Zero(6, nbTimeSamples));
                    }
                }
                BEMBody->SetImpulseResponseFunctionKu(bodyMotion, impulseResponseFunctionsKU);
            }
        }
    }

    void FrHydroDB::WaveDriftReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody) {

        // This subroutine reads the wave drift coefficients.

        BEMBody->SetWaveDrift();

        char buffer[20];
        auto listMode = {"surge", "sway", "heave", "roll", "pitch", "yaw"};

        auto data = reader.ReadDoubleArraySTD(path + "/freq");
        auto freqs = std::vector<double>(data[0]);

        BEMBody->GetWaveDrift()->SetFrequencies(freqs);
        BEMBody->GetWaveDrift()->SetAngles(GetWaveDirections(mathutils::RAD, NWU));

        auto nbWaveDirection = GetNbWaveDirections();

        for (std::string mode: listMode) {

            auto modePath = path + "/" + mode;

            if (reader.GroupExist(modePath)) {

                auto coeffs = std::vector<double>();

                for (unsigned int i_dir=0; i_dir<nbWaveDirection; ++i_dir) {

                    sprintf(buffer, "%d", i_dir);
                    auto idirPath = modePath + "/heading_" + buffer;

                    auto data = reader.ReadDoubleArraySTD(idirPath + "/data");
                    coeffs.insert(std::end(coeffs), std::begin(data[0]), std::end(data[0]));
                }
                BEMBody->GetWaveDrift()->AddData(mode, coeffs);
            }
        }
    }

    void FrHydroDB::HydrostaticReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody) {

        // This subroutine reads the hydrostatic matrix.

        mathutils::Matrix66<double> matrix = reader.ReadDoubleArray(path + "/StiffnessMatrix");
        BEMBody->SetStiffnessMatrix(matrix);
    }


    std::shared_ptr<FrHydroDB> make_hydrodynamic_database(std::string h5file) {

        // This subroutine reads the HDB from the *.HDB5 input file. All the necessary structures (FrHydroDB, FrBEMBody, etc.) are generated and initialized.

        return std::make_shared<FrHydroDB>(h5file);
    }

}  // end namespace frydom
