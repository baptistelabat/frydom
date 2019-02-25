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


//#include <boost/filesystem.hpp>

#include "yaml-cpp/yaml.h"

#include "FrHydroDB.h"
#include "FrHydroMapper.h"
#include "frydom/IO/FrHDF5.h"
#include "FrBEMBody.h"


using namespace mathutils;


// TODO: utiliser boost/multi_array.hpp a la place des vector<vector<Eigen::Matrix>>> ?????


namespace frydom {

//    void FrHydroDB::GenerateImpulseResponseFunctions(double tf, double dt) {
//
//        // Generate time informations
//        if (dt == 0.) {
//            // Ensuring a time step satisfying largely the shannon theorem (5x by security instead of the theoretical 2 ...)
//            dt = MU_2PI / (5. * GetMaxFrequency());  // TODO: non, on veut avoir un nombre d'echantillon en puissance de 2 !!!
//        }
//
//        // Registering the time discretization into the database
//        m_TimeDiscretization.SetMin(0.);
//        m_TimeDiscretization.SetMax(tf);
//        m_TimeDiscretization.SetStep(dt);
//
//        // Computing the Impulse response functions for every bodies
//        auto nbBody = GetNbBodies();
//        for (unsigned int iBody=0; iBody<nbBody; ++iBody) {
//            GetBody(iBody)->GenerateImpulseResponseFunctions();
//        }
//
//    }
//
//    void FrHydroDB::GenerateSpeedDependentIRF() {
//
//        assert(m_TimeDiscretization.GetMax() > m_TimeDiscretization.GetMin());
//        assert(m_TimeDiscretization.GetStep() > DBL_EPSILON);
//
//        // Computing the speed dependent term of the impulse response function for every bodies
//        auto nbBody = GetNbBodies();
//        for (unsigned int iBody=0; iBody<nbBody; iBody++) {
//            GetBody(iBody)->GenerateSpeedDependentIRF();
//        }
//    }
//
//
//    std::shared_ptr<FrHydroMapper> FrHydroDB::GetMapper() {
//        return std::make_shared<FrHydroMapper>(this);
//    }
//
//    FrHydroDB::FrHydroDB(std::string h5file) {
//
//        FrHDF5Reader reader;
//        reader.SetFilename(h5file);
//
//        auto GravityAcc = reader.ReadDouble("/GravityAcc");
//        this->SetGravityAcc(GravityAcc);
//
//        auto WaterDensity = reader.ReadDouble("/WaterDensity");
//        this->SetWaterDensity(WaterDensity);
//
//        auto NormalizationLength = reader.ReadDouble("/NormalizationLength");
//        this->SetNormalizationLength(NormalizationLength);
//
//        auto WaterDepth = reader.ReadDouble("/WaterDepth");
//        this->SetWaterDepth(WaterDepth);
//
//        auto NbBodies = reader.ReadInt("/NbBody");
//
//
//        std::string discretization_path = "/Discretizations";
//
//        // Reading frequency discretization
//        std::string frequency_discretization_path = discretization_path + "/Frequency";
//        auto NbFreq = reader.ReadInt(frequency_discretization_path + "/NbFrequencies");
//        auto MinFreq = reader.ReadDouble(frequency_discretization_path + "/MinFrequency");
//        auto MaxFreq = reader.ReadDouble(frequency_discretization_path + "/MaxFrequency");
//        this->SetFrequencyDiscretization(MinFreq, MaxFreq, (uint)NbFreq);
//
//        // Reading wave propagation direction discretization
//        std::string wave_direction_discretization_path = discretization_path + "/WaveDirections";
//        auto NbWaveDir = reader.ReadInt(wave_direction_discretization_path + "/NbWaveDirections");
//        auto MinWaveDir = reader.ReadDouble(wave_direction_discretization_path + "/MinAngle");
//        auto MaxWaveDir = reader.ReadDouble(wave_direction_discretization_path + "/MaxAngle");
//        this->SetWaveDirectionDiscretization(MinWaveDir, MaxWaveDir, (uint)NbWaveDir);
//
//        // Reading wave propagation direction discretization
//        std::string time_discretization_path = discretization_path + "/Time";
//        auto NbTimeSample = reader.ReadInt(time_discretization_path + "/NbTimeSample");
//        auto FinalTime = reader.ReadDouble(time_discretization_path + "/FinalTime");
//        this->SetTimeDiscretization(FinalTime, (uint)NbTimeSample);
//
//        // Getting data from body
//        std::string body_path("/Bodies/Body_");
//        std::string body_i_path, mode_path, imode_path;
//        char buffer [20];
//        for (unsigned int ibody=0; ibody<NbBodies; ++ibody) {
//
//            sprintf(buffer, "%d", ibody);
//            body_i_path = body_path + buffer;
//
//            auto BodyName = reader.ReadString(body_i_path + "/BodyName");
//            auto body = this->NewBody(BodyName);
//
//            auto BodyPosition = reader.ReadDoubleArray(body_i_path + "/BodyPosition");
//            body->SetBodyPosition(BodyPosition);
//
//            auto ID = reader.ReadInt(body_i_path + "/ID");
//            assert(body->GetID() == ID);
//
//            auto nbForceModes = reader.ReadInt(body_i_path + "/Modes/NbForceModes");
//
//            mode_path = body_i_path + "/Modes/ForceModes/Mode_";
//            std::string mode_type;
//            Eigen::Vector3d direction, point;
//            for (unsigned int iforce=0; iforce<nbForceModes; ++iforce) {
//                sprintf(buffer, "%d", iforce);
//                imode_path = mode_path + buffer;
//
//                // Building the force mode
//                FrBEMForceMode mode;
//
//                mode_type = reader.ReadString(imode_path + "/Type");
//                direction = reader.ReadDoubleArray(imode_path + "/Direction");
//
//                mode.SetDirection(direction);
//
//                if (mode_type == "ANGULAR") {
//                    point = reader.ReadDoubleArray(imode_path + "/Point");
//                    mode.SetTypeANGULAR();
//                    mode.SetPoint(point);
//                } else {
//                    mode.SetTypeLINEAR();
//                }
//
//                // Adding the mode to the BEMBody
//                body->AddForceMode(mode);
//
//            }  // end for iforce
//
//
//            auto nbMotionModes = reader.ReadInt(body_i_path + "/Modes/NbMotionModes");
//
//            mode_path = body_i_path + "/Modes/MotionModes/Mode_";
//            for (unsigned int imotion=0; imotion<nbForceModes; ++imotion) {
//                sprintf(buffer, "%d", imotion);
//                imode_path = mode_path + buffer;
//
//                // Building the force mode
//                FrBEMMotionMode mode;
//
//                mode_type = reader.ReadString(imode_path + "/Type");
//                direction = reader.ReadDoubleArray(imode_path + "/Direction");
//
//                mode.SetDirection(direction);
//
//                if (mode_type == "ANGULAR") {
//                    point = reader.ReadDoubleArray(imode_path + "/Point");
//                    mode.SetTypeANGULAR();
//                    mode.SetPoint(point);
//                } else {
//                    mode.SetTypeLINEAR();
//                }
//
//                // Adding the mode to the BEMBody
//                body->AddMotionMode(mode);
//
//            }  // end for imotion
//
//            // Reading the mesh
//            std::string ibody_mesh_path = body_i_path + "/Mesh";
//            auto nbVertices = reader.ReadInt(ibody_mesh_path + "/NbVertices");
//            auto vertices = reader.ReadDoubleArray(ibody_mesh_path + "/Vertices");
//
//            auto nbFaces = reader.ReadInt(ibody_mesh_path + "/NbFaces");
//            auto faces = reader.ReadIntArray(ibody_mesh_path + "/Faces");
//
//            // TODO: construire un objet maillage !!!!
//
//
//            body->Initialize();
//
//        }  // end for ibody
//
//        // READING THE HYDRODYNAMIC COEFFICIENTS
//        for (unsigned int ibody=0; ibody<NbBodies; ++ibody) {
//
//            sprintf(buffer, "%d", ibody);
//            body_i_path = body_path + buffer;
//            auto body = this->GetBody(ibody);
//
//            // Reading the excitation hydrodynamic coefficients
//            auto diffraction_path = body_i_path + "/Excitation/Diffraction";
//            auto froude_kylov_path = body_i_path + "/Excitation/FroudeKrylov";
//            std::string diffraction_wave_dir_path, fk_wave_dir_path;
//            for (unsigned int iwave_dir=0; iwave_dir<NbWaveDir; ++iwave_dir) {
//                sprintf(buffer, "/Angle_%d", iwave_dir);
//
//                // Reading diffraction coefficients
//                diffraction_wave_dir_path = diffraction_path + buffer;
//
//                auto diffraction_realCoeffs = reader.ReadDoubleArray(diffraction_wave_dir_path + "/RealCoeffs");
//                auto diffraction_imagCoeffs = reader.ReadDoubleArray(diffraction_wave_dir_path + "/ImagCoeffs");
//
//                Eigen::MatrixXcd diffractionCoeffs;
//                diffractionCoeffs = diffraction_realCoeffs + MU_JJ * diffraction_imagCoeffs;
//                body->SetDiffraction(iwave_dir, diffractionCoeffs);
//
//                // Reading Froude-Krylov coefficients
//                fk_wave_dir_path = froude_kylov_path + buffer;
//
//                auto fk_realCoeffs = reader.ReadDoubleArray(fk_wave_dir_path + "/RealCoeffs");
//                auto fk_imagCoeffs = reader.ReadDoubleArray(fk_wave_dir_path + "/ImagCoeffs");
//
//                Eigen::MatrixXcd froudeKrylovCoeffs;
//                froudeKrylovCoeffs = fk_realCoeffs + MU_JJ * fk_imagCoeffs;
//                body->SetFroudeKrylov(iwave_dir, froudeKrylovCoeffs);
//
//            }
//
//            // Reading the radiation coefficients
//            auto radiation_path = body_i_path + "/Radiation";
//            std::shared_ptr<FrBEMBody> body_motion;
//            for (unsigned int ibody_motion=0; ibody_motion<NbBodies; ++ibody_motion) {
//                sprintf(buffer, "/BodyMotion_%d", ibody_motion);
//
//                body_motion = this->GetBody(ibody_motion);
//
//                auto body_i_infinite_added_mass_path = radiation_path + buffer + "/InfiniteAddedMass";
//                auto infinite_added_mass = reader.ReadDoubleArray(body_i_infinite_added_mass_path);
//                body->SetInfiniteAddedMass(ibody_motion, infinite_added_mass);
//
//                auto body_i_added_mass_path = radiation_path + buffer + "/AddedMass";
//                auto body_i_radiation_damping_path = radiation_path + buffer + "/RadiationDamping";
//
//                auto body_i_impulse_response_function_path = radiation_path + buffer + "/ImpulseResponseFunction";
//                for (unsigned int imotion=0; imotion<body_motion->GetNbMotionMode(); ++imotion) {
//                    sprintf(buffer, "/DOF_%d", imotion);
//
//                    auto added_mass = reader.ReadDoubleArray(body_i_added_mass_path + buffer);
//                    body->SetAddedMass(ibody_motion, imotion, added_mass);
//
//                    auto radiation_damping = reader.ReadDoubleArray(body_i_radiation_damping_path + buffer);
//                    body->SetRadiationDamping(ibody_motion, imotion, radiation_damping);
//
//                    auto impulse_response_function = reader.ReadDoubleArray(body_i_impulse_response_function_path + buffer);
//                    body->SetImpulseResponseFunction(ibody_motion, imotion, impulse_response_function);
//
//                }  // end for imotion
//            }  // end ibody_motion
//
//            // Finalizing the HDB by computing different interpolators
//            body->Finalize();
//
//        }  // end for ibody (force)
//    }
//
//
//    void FrHydroDB::IntLoadResidual_Mv(const unsigned int off,
//                                       chrono::ChVectorDynamic<>& Res,
//                                       const chrono::ChVectorDynamic<>& w,
//                                       const double c) {
//        for (auto& body: m_Bodies) {
//
//            if (body->GetRadiationActive()) {
//                auto off_body = body->GetHydroBody()->GetOffset_w();
//                body->IntLoadResidual_Mv(off + off_body, Res, w, c);
//             }
//        }
//
//    }
//
//    /**
//    void FrHydroDB::VariablesFbIncrementMq() {
//        for (auto& body: m_Bodies) {
//            if (body->GetRadiationActive()) {
//                body->VariablesFbIncrementMq();
//            }
//        }
//    }
//    **/
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//    /// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    // ----------------------------------------------
    // FrDiscretization1D
    // ----------------------------------------------

    std::vector<double> FrDiscretization1D_::GetVector() const {
        return linspace<double>(m_xmin, m_xmax, m_nx);
    }

    void FrDiscretization1D_::SetStep(double delta) {
        m_nx = 1 + (unsigned int)((m_xmax - m_xmin) / delta);
    }

    double FrDiscretization1D_::GetStep() const {
        return (m_xmax-m_xmin) / double(m_nx-1);
    }

    // ----------------------------------------------------
    // FrHydroDB
    // ----------------------------------------------------

    void FrHydroDB_::SetWaveDirectionDiscretization(const double minAngle, const double maxAngle,
                                                    const unsigned int nbAngle) {
        m_waveDirectionDiscretization.SetMin(minAngle);
        m_waveDirectionDiscretization.SetMax(maxAngle);
        m_waveDirectionDiscretization.SetNbSample(nbAngle);
    }

    void FrHydroDB_::SetTimeDiscretization(const double finalTime, const unsigned int nbTimeSamples) {
        m_timeDiscretization.SetMin(0.);
        m_timeDiscretization.SetMax(finalTime);
        m_timeDiscretization.SetNbSample(nbTimeSamples);
    }

    void FrHydroDB_::SetFrequencyDiscretization(const double minFreq, const double maxFreq,
                                                const unsigned int nbFreq) {
        m_frequencyDiscretization.SetMin(minFreq);
        m_frequencyDiscretization.SetMax(maxFreq);
        m_frequencyDiscretization.SetNbSample(nbFreq);
    }

    std::vector<double> FrHydroDB_::GetWaveDirections(ANGLE_UNIT angleUnit, FRAME_CONVENTION fc) const {

        auto waveDir = m_waveDirectionDiscretization.GetVector();

        if (angleUnit == RAD) {
            for (auto& angle: waveDir) { angle *= DEG2RAD; }
        }

        if (IsNED(fc)) {
            for (auto& angle: waveDir) { angle = -angle; }
        }

        return waveDir;
    }

    FrBEMBody_* FrHydroDB_::NewBody(std::string bodyName) {
        m_bodies.push_back( std::make_unique<FrBEMBody_>(GetNbBodies(), bodyName, this));
        return m_bodies.back().get();
    }

    FrBEMBody_* FrHydroDB_::GetBody(std::shared_ptr<FrBody_> body) {
        return m_mapper->GetBEMBody(body.get());
    }

    FrBEMBody_* FrHydroDB_::GetBody(FrBody_* body) {
        return m_mapper->GetBEMBody(body);
    }

    FrBEMBody_* FrHydroDB_::GetBody(int ibody) {
        return m_bodies[ibody].get();
    }

    FrBody_* FrHydroDB_::GetBody(FrBEMBody_* body) {
        return m_mapper->GetBody(body);
    }

    FrHydroMapper_* FrHydroDB_::GetMapper() {
        return m_mapper.get();
    }

    void FrHydroDB_::Map(FrBEMBody_* BEMBody, FrBody_* body, std::shared_ptr<FrEquilibriumFrame_> eqFrame) {
        m_mapper->Map(BEMBody, body, eqFrame);
    }

    void FrHydroDB_::Map(int iBEMBody, FrBody_* body, std::shared_ptr<FrEquilibriumFrame_> eqFrame) {
        m_mapper->Map(m_bodies[iBEMBody].get(), body, eqFrame);
    }

    FrHydroDB_::FrHydroDB_(std::string h5file) {

        /// Constructor of the class.
        /// This subroutine generates the FrHydroDB_ object from the *.HDB5 input file.

        /// Object for reading *.HDB5 input file.
        FrHDF5Reader reader;
        reader.SetFilename(h5file);

        /// Reading environmental data and the number of bodies.
        m_gravityAcc = reader.ReadDouble("/GravityAcc");
        m_waterDensity = reader.ReadDouble("/WaterDensity");
        m_normalizationLength = reader.ReadDouble("/NormalizationLength");
        m_waterDepth = reader.ReadDouble("/WaterDepth");
        m_nbody = reader.ReadInt("/NbBody");

        /// Creation of the mapper object for hydrodynamic bodies.
        m_mapper = std::make_unique<FrHydroMapper_>();

        /// ----> Reading discretization path
        std::string discretization_path = "/Discretizations";

        /// -----------> Reading frequency path
        std::string frequency_discretization_path = discretization_path + "/Frequency";
        auto NbFreq = reader.ReadInt(frequency_discretization_path + "/NbFrequencies"); // Number of frequencies.
        auto MinFreq = reader.ReadDouble(frequency_discretization_path + "/MinFrequency"); // Minimum frequency.
        auto MaxFreq = reader.ReadDouble(frequency_discretization_path + "/MaxFrequency"); // Maximum frequency.
        this->SetFrequencyDiscretization(MinFreq, MaxFreq, (uint)NbFreq); // Settings the frequency discretization.

        /// ------------> Reading waves directions
        std::string wave_direction_discretization_path = discretization_path + "/WaveDirections";
        auto NbWaveDir = reader.ReadInt(wave_direction_discretization_path + "/NbWaveDirections"); // Number of wave directions.
        auto MinWaveDir = reader.ReadDouble(wave_direction_discretization_path + "/MinAngle"); // Minimum wave direction.
        auto MaxWaveDir = reader.ReadDouble(wave_direction_discretization_path + "/MaxAngle"); // Maximum wave direction.
        this->SetWaveDirectionDiscretization(MinWaveDir, MaxWaveDir, (uint)NbWaveDir); // Settings the wave direction discretization.

        /// -------------> Reading time discretization
        std::string time_discretization_path = discretization_path + "/Time";
        auto NbTimeSample = reader.ReadInt(time_discretization_path + "/NbTimeSample"); // Number of time steps.
        auto FinalTime = reader.ReadDouble(time_discretization_path + "/FinalTime"); // Duration.
        this->SetTimeDiscretization(FinalTime, (uint)NbTimeSample); // Settings the time discretization.

        /// -------------> Reading data from Body
        char buffer[20];
        std::string bodyPath = "/Bodies/Body_";

        /// Loop over all the bodies subject to hydrodynamic loads.
        for (unsigned int ibody=0; ibody<m_nbody; ++ibody) {

            sprintf(buffer, "%d", ibody);
            auto ibodyPath = bodyPath + buffer;
            auto bodyName = reader.ReadString(ibodyPath + "/BodyName");

            /// Creation of a new FrBEMBody.
            auto BEMBody = this->NewBody(bodyName);

            /// Initialization from the *.HDB5 input file.
            Position bodyPosition = reader.ReadDoubleArray(ibodyPath + "/BodyPosition");
            BEMBody->SetPosition(bodyPosition);

            auto ID = reader.ReadInt(ibodyPath + "/ID");
            assert(BEMBody->GetID() == ID);

            /// Reading the modes of a body.
            this->ModeReader(reader, ibodyPath, BEMBody);

            /// Allocation of the arrays for the hdb.
            BEMBody->Initialize();

        }

        /// -----------> Reading the hydrodynamic coefficients
        for (unsigned int ibody=0; ibody<m_nbody; ++ibody) {

            sprintf(buffer, "%d", ibody);
            auto ibodyPath = bodyPath + buffer;

            auto BEMBody = this->GetBody(ibody);

            /// Reading of the excitation loads.
            this->ExcitationReader(reader, ibodyPath, BEMBody);

            /// Reading of the radiation coefficients (infinite added mass, impulse response functions).
            this->RadiationReader(reader, ibodyPath, BEMBody);

            /// Reading the wave drift coefficients.
            if (reader.GroupExist(ibodyPath + "/WaveDrift")) {
                this->WaveDriftReader(reader, ibodyPath + "/WaveDrift", BEMBody);
            }

            /// Reading the hydrostatic matrix.
            if (reader.GroupExist(ibodyPath + "/Hydrostatic")) {
                this->HydrostaticReader(reader, ibodyPath + "/Hydrostatic", BEMBody);
            }

            /// Interpolation of the excitation loads with respect to the wave direction.
            this->GetBody(ibody)->Finalize();
        }
    }

    void FrHydroDB_::ModeReader(FrHDF5Reader& reader, std::string path, FrBEMBody_* BEMBody) {

        /// This subroutine reads the modes of a body.

        char buffer[20];

        // ----- Force Mode ----------------------

        auto nbForceModes = reader.ReadInt(path + "/Modes/NbForceModes");

        auto modePath = path + "/Modes/ForceModes/Mode_";

        for (unsigned int iforce=0; iforce<nbForceModes; ++iforce) {

            sprintf(buffer, "%d", iforce);
            auto imodePath = modePath + buffer;

            // Building the force mode
            FrBEMForceMode_ mode;

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

            FrBEMMotionMode_ mode;

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

    void FrHydroDB_::ExcitationReader(FrHDF5Reader& reader, std::string path, FrBEMBody_* BEMBody) {

        /// This subroutine reads the excitation loads from the *.HDB5 input file.

        char buffer[20];

        auto diffractionPath = path + "/Excitation/Diffraction";
        auto froudeKrylovPath = path + "/Excitation/FroudeKrylov/";

        for (unsigned int iwaveDir=0; iwaveDir < GetNbWaveDirections(); ++iwaveDir) {

            sprintf(buffer, "/Angle_%d", iwaveDir);

            /// -> Diffraction loads.
            auto diffractionWaveDirPath = diffractionPath + buffer;
            auto diffractionRealCoeffs = reader.ReadDoubleArray(diffractionWaveDirPath + "/RealCoeffs");
            auto diffractionImagCoeffs = reader.ReadDoubleArray(diffractionWaveDirPath + "/ImagCoeffs");

            Eigen::MatrixXcd diffractionCoeffs;
            diffractionCoeffs = diffractionRealCoeffs + MU_JJ * diffractionImagCoeffs; // In complex.
            BEMBody->SetDiffraction(iwaveDir, diffractionCoeffs);

            /// -> Froude-Krylov loads.
            auto froudeKrylovWaveDirPath = froudeKrylovPath + buffer;
            auto froudeKrylovRealCoeffs = reader.ReadDoubleArray(froudeKrylovWaveDirPath + "/RealCoeffs");
            auto froudeKrylovImagCoeffs = reader.ReadDoubleArray(froudeKrylovWaveDirPath + "/ImagCoeffs");

            Eigen::MatrixXcd froudeKrylovCoeffs;
            froudeKrylovCoeffs = froudeKrylovRealCoeffs + MU_JJ * froudeKrylovImagCoeffs; // In complex.
            BEMBody->SetFroudeKrylov(iwaveDir, froudeKrylovCoeffs);

        }

        /// Computation of the excitation loads the diffraction and Froude-Krylov loads.
        BEMBody->ComputeExcitation();
    }

    void FrHydroDB_::RadiationReader(FrHDF5Reader& reader, std::string path, FrBEMBody_* BEMBody) {

        /// This subroutine reads the radiation coefficients from the *.HDB5 input file.

        char buffer[20];

        auto radiationPath = path + "/Radiation";

        for (unsigned int ibodyMotion=0; ibodyMotion < m_nbody; ++ibodyMotion) {

            sprintf(buffer, "/BodyMotion_%d", ibodyMotion);

            auto bodyMotion = this->GetBody(ibodyMotion);

            /// Reading the infinite added mass matrix for the body.
            auto infiniteAddedMassPath = radiationPath + buffer + "/InfiniteAddedMass";
            auto infiniteAddedMass = reader.ReadDoubleArray(infiniteAddedMassPath);
            BEMBody->SetInfiniteAddedMass(bodyMotion, infiniteAddedMass);

            /// Reading the impulse response functions.
            auto IRFPath = radiationPath + buffer + "/ImpulseResponseFunctionK";
            auto IRFUPath = radiationPath + buffer + "/ImpulseResponseFunctionKU";

            std::vector<Eigen::MatrixXd> impulseResponseFunctionsK;
            std::vector<Eigen::MatrixXd> impulseResponseFunctionsKU;

            for (unsigned int imotion=0; imotion<bodyMotion->GetNbMotionMode(); ++imotion) {
                sprintf(buffer, "/DOF_%d", imotion);
                impulseResponseFunctionsK.push_back(reader.ReadDoubleArray(IRFPath + buffer));
            }
            BEMBody->SetImpulseResponseFunctionK(bodyMotion, impulseResponseFunctionsK);

            /// Reading the impulse response functions in case of forward speed.
            if (reader.GroupExist(IRFUPath)) {

                for (unsigned int imotion=0; imotion<bodyMotion->GetNbMotionMode(); ++imotion) {
                    sprintf(buffer, "/DOF_%d", imotion);
                    impulseResponseFunctionsKU.push_back(reader.ReadDoubleArray(IRFUPath + buffer));
                }
                BEMBody->SetImpulseResponseFunctionKu(bodyMotion, impulseResponseFunctionsKU);
            }
        }
    }

    void FrHydroDB_::WaveDriftReader(FrHDF5Reader& reader, std::string path, FrBEMBody_* BEMBody) {

        /// This subroutine reads the wave drift coefficients.

        BEMBody->SetWaveDrift();

        char buffer[20];
        auto listMode = {"surge", "sway", "heave", "roll", "pitch", "yaw"};

        auto data = reader.ReadDoubleArraySTD(path + "/freq");
        auto freqs = std::vector<double>(data[0]);

        BEMBody->GetWaveDrift()->SetFrequencies(freqs);
        BEMBody->GetWaveDrift()->SetAngles(GetWaveDirections(RAD, NWU));

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

    void FrHydroDB_::HydrostaticReader(FrHDF5Reader& reader, std::string path, FrBEMBody_* BEMBody) {

        /// This subroutine reads the hydrostatic matrix.

        Matrix66<double> matrix = reader.ReadDoubleArray(path + "/StiffnessMatrix");
        BEMBody->SetStiffnessMatrix(matrix);
    }


    std::shared_ptr<FrHydroDB_> make_hydrodynamic_database(std::string h5file) {

        /// This subroutine reads the HDB from the *.HDB5 input file. All the necessary structures (FrHydroDB, FrBEMBody, etc.) are generated and initialized.

        return std::make_shared<FrHydroDB_>(h5file);
    }

}  // end namespace frydom
