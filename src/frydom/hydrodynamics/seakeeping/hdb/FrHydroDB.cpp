//
// Created by frongere on 17/10/17.
//

//#include <boost/filesystem.hpp>

#include "yaml-cpp/yaml.h"

#include "frydom/core/FrHydroBody.h" // TODO : a retirer

#include "FrHydroDB.h"
#include "FrBEMBody.h"
#include "FrHydroMapper.h"
#include "frydom/IO/FrHDF5.h"




using namespace mathutils;


// TODO: utiliser boost/multi_array.hpp a la place des vector<vector<Eigen::Matrix>>> ?????


namespace frydom {

    void FrHydroDB::GenerateImpulseResponseFunctions(double tf, double dt) {

        // Generate time informations
        if (dt == 0.) {
            // Ensuring a time step satisfying largely the shannon theorem (5x by security instead of the theoretical 2 ...)
            dt = MU_2PI / (5. * GetMaxFrequency());  // TODO: non, on veut avoir un nombre d'echantillon en puissance de 2 !!!
        }

        // Registering the time discretization into the database
        m_TimeDiscretization.SetMin(0.);
        m_TimeDiscretization.SetMax(tf);
        m_TimeDiscretization.SetStep(dt);

        // Computing the Impulse response functions for every bodies
        auto nbBody = GetNbBodies();
        for (unsigned int iBody=0; iBody<nbBody; ++iBody) {
            GetBody(iBody)->GenerateImpulseResponseFunctions();
        }

    }

    void FrHydroDB::GenerateSpeedDependentIRF() {

        assert(m_TimeDiscretization.GetMax() > m_TimeDiscretization.GetMin());
        assert(m_TimeDiscretization.GetStep() > DBL_EPSILON);

        // Computing the speed dependent term of the impulse response function for every bodies
        auto nbBody = GetNbBodies();
        for (unsigned int iBody=0; iBody<nbBody; iBody++) {
            GetBody(iBody)->GenerateSpeedDependentIRF();
        }
    }


    std::shared_ptr<FrHydroMapper> FrHydroDB::GetMapper() {
        return std::make_shared<FrHydroMapper>(this);
    }

    FrHydroDB::FrHydroDB(std::string h5file) {

        FrHDF5Reader reader;
        reader.SetFilename(h5file);

        auto GravityAcc = reader.ReadDouble("/GravityAcc");
        this->SetGravityAcc(GravityAcc);

        auto WaterDensity = reader.ReadDouble("/WaterDensity");
        this->SetWaterDensity(WaterDensity);

        auto NormalizationLength = reader.ReadDouble("/NormalizationLength");
        this->SetNormalizationLength(NormalizationLength);

        auto WaterDepth = reader.ReadDouble("/WaterDepth");
        this->SetWaterDepth(WaterDepth);

        auto NbBodies = reader.ReadInt("/NbBody");


        std::string discretization_path = "/Discretizations";

        // Reading frequency discretization
        std::string frequency_discretization_path = discretization_path + "/Frequency";
        auto NbFreq = reader.ReadInt(frequency_discretization_path + "/NbFrequencies");
        auto MinFreq = reader.ReadDouble(frequency_discretization_path + "/MinFrequency");
        auto MaxFreq = reader.ReadDouble(frequency_discretization_path + "/MaxFrequency");
        this->SetFrequencyDiscretization(MinFreq, MaxFreq, (uint)NbFreq);

        // Reading wave propagation direction discretization
        std::string wave_direction_discretization_path = discretization_path + "/WaveDirections";
        auto NbWaveDir = reader.ReadInt(wave_direction_discretization_path + "/NbWaveDirections");
        auto MinWaveDir = reader.ReadDouble(wave_direction_discretization_path + "/MinAngle");
        auto MaxWaveDir = reader.ReadDouble(wave_direction_discretization_path + "/MaxAngle");
        this->SetWaveDirectionDiscretization(MinWaveDir, MaxWaveDir, (uint)NbWaveDir);

        // Reading wave propagation direction discretization
        std::string time_discretization_path = discretization_path + "/Time";
        auto NbTimeSample = reader.ReadInt(time_discretization_path + "/NbTimeSample");
        auto FinalTime = reader.ReadDouble(time_discretization_path + "/FinalTime");
        this->SetTimeDiscretization(FinalTime, (uint)NbTimeSample);

        // Getting data from body
        std::string body_path("/Bodies/Body_");
        std::string body_i_path, mode_path, imode_path;
        char buffer [20];
        for (unsigned int ibody=0; ibody<NbBodies; ++ibody) {

            sprintf(buffer, "%d", ibody);
            body_i_path = body_path + buffer;

            auto BodyName = reader.ReadString(body_i_path + "/BodyName");
            auto body = this->NewBody(BodyName);

            auto BodyPosition = reader.ReadDoubleArray(body_i_path + "/BodyPosition");
            body->SetBodyPosition(BodyPosition);

            auto ID = reader.ReadInt(body_i_path + "/ID");
            assert(body->GetID() == ID);

            auto nbForceModes = reader.ReadInt(body_i_path + "/Modes/NbForceModes");

            mode_path = body_i_path + "/Modes/ForceModes/Mode_";
            std::string mode_type;
            Eigen::Vector3d direction, point;
            for (unsigned int iforce=0; iforce<nbForceModes; ++iforce) {
                sprintf(buffer, "%d", iforce);
                imode_path = mode_path + buffer;

                // Building the force mode
                FrBEMForceMode mode;

                mode_type = reader.ReadString(imode_path + "/Type");
                direction = reader.ReadDoubleArray(imode_path + "/Direction");

                mode.SetDirection(direction);

                if (mode_type == "ANGULAR") {
                    point = reader.ReadDoubleArray(imode_path + "/Point");
                    mode.SetTypeANGULAR();
                    mode.SetPoint(point);
                } else {
                    mode.SetTypeLINEAR();
                }

                // Adding the mode to the BEMBody
                body->AddForceMode(mode);

            }  // end for iforce


            auto nbMotionModes = reader.ReadInt(body_i_path + "/Modes/NbMotionModes");

            mode_path = body_i_path + "/Modes/MotionModes/Mode_";
            for (unsigned int imotion=0; imotion<nbForceModes; ++imotion) {
                sprintf(buffer, "%d", imotion);
                imode_path = mode_path + buffer;

                // Building the force mode
                FrBEMMotionMode mode;

                mode_type = reader.ReadString(imode_path + "/Type");
                direction = reader.ReadDoubleArray(imode_path + "/Direction");

                mode.SetDirection(direction);

                if (mode_type == "ANGULAR") {
                    point = reader.ReadDoubleArray(imode_path + "/Point");
                    mode.SetTypeANGULAR();
                    mode.SetPoint(point);
                } else {
                    mode.SetTypeLINEAR();
                }

                // Adding the mode to the BEMBody
                body->AddMotionMode(mode);

            }  // end for imotion

            // Reading the mesh
            std::string ibody_mesh_path = body_i_path + "/Mesh";
            auto nbVertices = reader.ReadInt(ibody_mesh_path + "/NbVertices");
            auto vertices = reader.ReadDoubleArray(ibody_mesh_path + "/Vertices");

            auto nbFaces = reader.ReadInt(ibody_mesh_path + "/NbFaces");
            auto faces = reader.ReadIntArray(ibody_mesh_path + "/Faces");

            // TODO: construire un objet maillage !!!!


            body->Initialize();

        }  // end for ibody

        // READING THE HYDRODYNAMIC COEFFICIENTS
        for (unsigned int ibody=0; ibody<NbBodies; ++ibody) {

            sprintf(buffer, "%d", ibody);
            body_i_path = body_path + buffer;
            auto body = this->GetBody(ibody);

            // Reading the excitation hydrodynamic coefficients
            auto diffraction_path = body_i_path + "/Excitation/Diffraction";
            auto froude_kylov_path = body_i_path + "/Excitation/FroudeKrylov";
            std::string diffraction_wave_dir_path, fk_wave_dir_path;
            for (unsigned int iwave_dir=0; iwave_dir<NbWaveDir; ++iwave_dir) {
                sprintf(buffer, "/Angle_%d", iwave_dir);

                // Reading diffraction coefficients
                diffraction_wave_dir_path = diffraction_path + buffer;

                auto diffraction_realCoeffs = reader.ReadDoubleArray(diffraction_wave_dir_path + "/RealCoeffs");
                auto diffraction_imagCoeffs = reader.ReadDoubleArray(diffraction_wave_dir_path + "/ImagCoeffs");

                Eigen::MatrixXcd diffractionCoeffs;
                diffractionCoeffs = diffraction_realCoeffs + MU_JJ * diffraction_imagCoeffs;
                body->SetDiffraction(iwave_dir, diffractionCoeffs);

                // Reading Froude-Krylov coefficients
                fk_wave_dir_path = froude_kylov_path + buffer;

                auto fk_realCoeffs = reader.ReadDoubleArray(fk_wave_dir_path + "/RealCoeffs");
                auto fk_imagCoeffs = reader.ReadDoubleArray(fk_wave_dir_path + "/ImagCoeffs");

                Eigen::MatrixXcd froudeKrylovCoeffs;
                froudeKrylovCoeffs = fk_realCoeffs + MU_JJ * fk_imagCoeffs;
                body->SetFroudeKrylov(iwave_dir, froudeKrylovCoeffs);

            }

            // Reading the radiation coefficients
            auto radiation_path = body_i_path + "/Radiation";
            std::shared_ptr<FrBEMBody> body_motion;
            for (unsigned int ibody_motion=0; ibody_motion<NbBodies; ++ibody_motion) {
                sprintf(buffer, "/BodyMotion_%d", ibody_motion);

                body_motion = this->GetBody(ibody_motion);

                auto body_i_infinite_added_mass_path = radiation_path + buffer + "/InfiniteAddedMass";
                auto infinite_added_mass = reader.ReadDoubleArray(body_i_infinite_added_mass_path);
                body->SetInfiniteAddedMass(ibody_motion, infinite_added_mass);

                auto body_i_added_mass_path = radiation_path + buffer + "/AddedMass";
                auto body_i_radiation_damping_path = radiation_path + buffer + "/RadiationDamping";

                auto body_i_impulse_response_function_path = radiation_path + buffer + "/ImpulseResponseFunction";
                for (unsigned int imotion=0; imotion<body_motion->GetNbMotionMode(); ++imotion) {
                    sprintf(buffer, "/DOF_%d", imotion);

                    auto added_mass = reader.ReadDoubleArray(body_i_added_mass_path + buffer);
                    body->SetAddedMass(ibody_motion, imotion, added_mass);

                    auto radiation_damping = reader.ReadDoubleArray(body_i_radiation_damping_path + buffer);
                    body->SetRadiationDamping(ibody_motion, imotion, radiation_damping);

                    auto impulse_response_function = reader.ReadDoubleArray(body_i_impulse_response_function_path + buffer);
                    body->SetImpulseResponseFunction(ibody_motion, imotion, impulse_response_function);

                }  // end for imotion
            }  // end ibody_motion

            // Finalizing the HDB by computing different interpolators
            body->Finalize();

        }  // end for ibody (force)
    }


    void FrHydroDB::IntLoadResidual_Mv(const unsigned int off,
                                       chrono::ChVectorDynamic<>& Res,
                                       const chrono::ChVectorDynamic<>& w,
                                       const double c) {
        for (auto& body: m_Bodies) {

            if (body->GetRadiationActive()) {
                auto off_body = body->GetHydroBody()->GetOffset_w();
                body->IntLoadResidual_Mv(off + off_body, Res, w, c);
             }
        }

    }

    /**
    void FrHydroDB::VariablesFbIncrementMq() {
        for (auto& body: m_Bodies) {
            if (body->GetRadiationActive()) {
                body->VariablesFbIncrementMq();
            }
        }
    }
    **/















    ///////////////// REFACTORING ----------------->>>>>>>>>>>>>>>>>>>

    bool FrBEMBodyMapper_::AddEntry(FrBEMBody_* bemBody, FrBody_* frydomBody) {
        bool success = true;
//        success *= m_namedBEMBodyMap.insert({bemBody->GetName(), bemBody}).second;
        success *= m_BEMToFrydomBodyMap.insert({bemBody, frydomBody}).second;
        success *= m_FrydomToBEMBodyMap.insert({frydomBody, bemBody}).second;
        return success;
    }

    FrBody_* FrBEMBodyMapper_::GetFrydomBody(FrBEMBody_* bemBody) const {
        return m_BEMToFrydomBodyMap.at(bemBody);
    }

    FrBEMBody_* FrBEMBodyMapper_::GetBEMBody(FrBody_* frydomBody) const {
        return m_FrydomToBEMBodyMap.at(frydomBody);
    }

    unsigned long FrBEMBodyMapper_::GetNbMappedBodies() const {
        auto size = m_FrydomToBEMBodyMap.size();
        assert(m_BEMToFrydomBodyMap.size() == size);
        return size;
    }






    FrHydroDB_::FrHydroDB_(std::string hdb5File) {

        FrHDF5Reader reader;
        reader.SetFilename(hdb5File);

        m_GravityAcc = reader.ReadDouble("/GravityAcc");
        m_WaterDensity = reader.ReadDouble("/WaterDensity");
        m_NormalizationLength = reader.ReadDouble("/NormalizationLength");
        m_WaterDepth = reader.ReadDouble("/WaterDepth");

        auto NbBodies = reader.ReadInt("/NbBody");

        std::string discretization_path = "/Discretizations";

        // Reading frequency discretization
        std::string frequency_discretization_path = discretization_path + "/Frequency";
        m_FrequencyDiscretization.SetNbSample((uint)reader.ReadInt(frequency_discretization_path + "/NbFrequencies"));
        m_FrequencyDiscretization.SetMin(reader.ReadDouble(frequency_discretization_path + "/MinFrequency"));
        m_FrequencyDiscretization.SetMax(reader.ReadDouble(frequency_discretization_path + "/MaxFrequency"));

        // Reading wave propagation direction discretization
        std::string wave_direction_discretization_path = discretization_path + "/WaveDirections";
        m_WaveDirectionDiscretization.SetNbSample((uint)reader.ReadInt(wave_direction_discretization_path + "/NbWaveDirections"));
        m_WaveDirectionDiscretization.SetMin(reader.ReadDouble(wave_direction_discretization_path + "/MinAngle"));
        m_WaveDirectionDiscretization.SetMax(reader.ReadDouble(wave_direction_discretization_path + "/MaxAngle"));

        // Reading wave propagation direction discretization
        std::string time_discretization_path = discretization_path + "/Time";
        m_TimeDiscretization.SetNbSample((uint)reader.ReadInt(time_discretization_path + "/NbTimeSample"));
        m_TimeDiscretization.SetMin(0.);
        m_TimeDiscretization.SetMax(reader.ReadDouble(time_discretization_path + "/FinalTime"));


        // Getting data from body
        std::string body_path("/Bodies/Body_");
        std::string body_i_path, mode_path, imode_path;
        char buffer [20];
        for (unsigned int ibody=0; ibody<NbBodies; ++ibody) {

            sprintf(buffer, "%d", ibody);
            body_i_path = body_path + buffer;

            auto BodyName = reader.ReadString(body_i_path + "/BodyName");
            FrBEMBody_* body = NewBEMBody(BodyName);

            Position BodyPosition = reader.ReadDoubleArray(body_i_path + "/BodyPosition");
            body->SetWorldPosition(BodyPosition);

//            auto ID = reader.ReadInt(body_i_path + "/ID");
//            assert(body->GetID() == ID);

            auto nbForceModes = reader.ReadInt(body_i_path + "/Modes/NbForceModes");

            mode_path = body_i_path + "/Modes/ForceModes/Mode_";
            std::string mode_type;

            Direction direction;
            Position point;
            for (unsigned int iforce=0; iforce < nbForceModes; ++iforce) {
                sprintf(buffer, "%d", iforce);
                imode_path = mode_path + buffer;

                // Building the force mode
                FrBEMForceMode mode;

                mode_type = reader.ReadString(imode_path + "/Type");
                direction = reader.ReadDoubleArray(imode_path + "/Direction");

                mode.SetDirection(direction);

                if (mode_type == "ANGULAR") {
                    point = reader.ReadDoubleArray(imode_path + "/Point");
                    mode.SetTypeANGULAR();
                    mode.SetPoint(point);
                } else {
                    mode.SetTypeLINEAR();
                }

                // Adding the mode to the BEMBody
                body->AddForceMode(mode);

            }  // end for iforce


            auto nbMotionModes = reader.ReadInt(body_i_path + "/Modes/NbMotionModes");

            mode_path = body_i_path + "/Modes/MotionModes/Mode_";
            for (unsigned int imotion=0; imotion<nbForceModes; ++imotion) {
                sprintf(buffer, "%d", imotion);
                imode_path = mode_path + buffer;

                // Building the force mode
                FrBEMMotionMode mode;

                mode_type = reader.ReadString(imode_path + "/Type");
                direction = reader.ReadDoubleArray(imode_path + "/Direction");

                mode.SetDirection(direction);

                if (mode_type == "ANGULAR") {
                    point = reader.ReadDoubleArray(imode_path + "/Point");
                    mode.SetTypeANGULAR();
                    mode.SetPoint(point);
                } else {
                    mode.SetTypeLINEAR();
                }

                // Adding the mode to the BEMBody
                body->AddMotionMode(mode);

            }  // end for imotion

            // Reading the mesh
            std::string ibody_mesh_path = body_i_path + "/Mesh";
            auto nbVertices = reader.ReadInt(ibody_mesh_path + "/NbVertices");
            auto vertices = reader.ReadDoubleArray(ibody_mesh_path + "/Vertices");

            auto nbFaces = reader.ReadInt(ibody_mesh_path + "/NbFaces");
            auto faces = reader.ReadIntArray(ibody_mesh_path + "/Faces");

            // TODO: construire un objet maillage !!!!


            body->Initialize();

        }  // end for ibody


        // READING THE HYDRODYNAMIC COEFFICIENTS
        unsigned int nbWaveDir = m_WaveDirectionDiscretization.GetNbSample();
        for (unsigned int ibody=0; ibody<NbBodies; ++ibody) {

            sprintf(buffer, "%d", ibody);
            body_i_path = body_path + buffer;
            auto body = m_bodies[ibody].get();


            // Reading the excitation hydrodynamic coefficients
            auto diffraction_path = body_i_path + "/Excitation/Diffraction";
            auto froude_kylov_path = body_i_path + "/Excitation/FroudeKrylov";
            std::string diffraction_wave_dir_path, fk_wave_dir_path;
            for (unsigned int iwave_dir=0; iwave_dir<nbWaveDir; ++iwave_dir) {
                sprintf(buffer, "/Angle_%d", iwave_dir);

                // Reading diffraction coefficients
                diffraction_wave_dir_path = diffraction_path + buffer;

                auto diffraction_realCoeffs = reader.ReadDoubleArray(diffraction_wave_dir_path + "/RealCoeffs");
                auto diffraction_imagCoeffs = reader.ReadDoubleArray(diffraction_wave_dir_path + "/ImagCoeffs");

                Eigen::MatrixXcd diffractionCoeffs;
                diffractionCoeffs = diffraction_realCoeffs + MU_JJ * diffraction_imagCoeffs;
                body->SetDiffraction(iwave_dir, diffractionCoeffs);

                // Reading Froude-Krylov coefficients
                fk_wave_dir_path = froude_kylov_path + buffer;

                auto fk_realCoeffs = reader.ReadDoubleArray(fk_wave_dir_path + "/RealCoeffs");
                auto fk_imagCoeffs = reader.ReadDoubleArray(fk_wave_dir_path + "/ImagCoeffs");

                Eigen::MatrixXcd froudeKrylovCoeffs;
                froudeKrylovCoeffs = fk_realCoeffs + MU_JJ * fk_imagCoeffs;
                body->SetFroudeKrylov(iwave_dir, froudeKrylovCoeffs);

            }

            // Reading the radiation coefficients
            auto radiation_path = body_i_path + "/Radiation";
            FrBEMBody_* body_motion;
            for (unsigned int ibody_motion=0; ibody_motion<NbBodies; ++ibody_motion) {
                sprintf(buffer, "/BodyMotion_%d", ibody_motion);

//                body_motion = GetBody(ibody_motion);
                body_motion = m_bodies[ibody_motion].get();

                auto body_i_infinite_added_mass_path = radiation_path + buffer + "/InfiniteAddedMass";
                auto infinite_added_mass = reader.ReadDoubleArray(body_i_infinite_added_mass_path);
                body->SetInfiniteAddedMass(body_motion, infinite_added_mass);

                auto body_i_added_mass_path = radiation_path + buffer + "/AddedMass";
                auto body_i_radiation_damping_path = radiation_path + buffer + "/RadiationDamping";

                auto body_i_impulse_response_function_path = radiation_path + buffer + "/ImpulseResponseFunction";
                for (unsigned int imotion=0; imotion<body_motion->GetNbMotionMode(); ++imotion) {
                    sprintf(buffer, "/DOF_%d", imotion);

                    auto added_mass = reader.ReadDoubleArray(body_i_added_mass_path + buffer);
                    body->SetAddedMass(body_motion, imotion, added_mass);

                    auto radiation_damping = reader.ReadDoubleArray(body_i_radiation_damping_path + buffer);
                    body->SetRadiationDamping(body_motion, imotion, radiation_damping);

                    auto impulse_response_function = reader.ReadDoubleArray(body_i_impulse_response_function_path + buffer);
                    body->SetImpulseResponseFunction(body_motion, imotion, impulse_response_function);

                }  // end for imotion
            }  // end ibody_motion

            // Finalizing the HDB by computing different interpolators
            body->Finalize();

        }  // end for ibody (force)

    }

    void FrHydroDB_::SetCutoffTime(double cutoffTime) {
        m_cutoffTime = cutoffTime;
    }

    void FrHydroDB_::Bind(std::string bemBodyName, FrBody_* frydomBody) {
        m_bodyMapper->AddEntry(GetBEMBody(std::move(bemBodyName)), frydomBody);
    }

    FrBEMBody_* FrHydroDB_::GetBEMBody(FrBody_* frydomBody) {
        return m_bodyMapper->GetBEMBody(frydomBody);
    }

    FrBEMBody_* FrHydroDB_::GetBEMBody(std::string bemBodyName) {
        return m_namedBodyMap.at(bemBodyName).second;
    }

    FrBody_* FrHydroDB_::GetFrydomBody(FrBEMBody_* bemBody) {
        return m_bodyMapper->GetFrydomBody(bemBody);
    }

//    const FrBody_* FrHydroDB_::GetFrydomBody(const FrBEMBody_* bemBody) const {
//        return m_bodyMapper->GetFrydomBody(bemBody);
//    }

    bool FrHydroDB_::IsFullyConnected() const {
        return GetNbBodies() == m_bodyMapper->GetNbMappedBodies();
    }

    void FrHydroDB_::Initialize() {
        if (c_isInitialized) return;

        assert(IsFullyConnected());

        // TODO : faire le calcul des reponses impulsionnelles par expl...
        GenerateImpulseResponseFunctions();


        // Filtering the Impulse responses functions
        if (m_cutoffTime <= 0.) EstimateCutoffTime(); // The cutoff time is automatically estimated by FRyDoM
        FilterImpulseResponseFunctions();


        // AUTRE ?


        c_isInitialized = true;
    }


    // BEM body iterator

    FrHydroDB_::BEMBodyIter FrHydroDB_::begin_body() {
        return m_bodies.begin();
    }

    FrHydroDB_::BEMBodyIter FrHydroDB_::end_body() {
        return m_bodies.end();
    }

//    FrHydroDB_::BEMBodyConstIter FrHydroDB_::begin_body() const {
//        return m_bodies.cbegin();
//    }
//
//    FrHydroDB_::BEMBodyConstIter FrHydroDB_::end_body() const {
//        return m_bodies.cend();
//    }




    unsigned long FrHydroDB_::GetBodyIndex(const FrBEMBody_* bemBody) {
        return m_namedBodyMap.at(bemBody->GetName()).first;
    }

    void FrHydroDB_::EstimateCutoffTime() {
        // give a damping level approximation Cutoff Tolerance, as a percentage error
        // relative to the largest damping level given


        // Getting the maximum wave damping damping
        double maxDamping = 0.;

        auto bodyIter = begin_body();
        for (; bodyIter != end_body(); bodyIter++) {
            maxDamping = std::max(maxDamping, (*bodyIter)->GetMaxDamping());
        }


    }















    std::shared_ptr<FrHydroDB_> LoadHDB5_(std::string hdb5File) {
        return std::make_shared<FrHydroDB_>(hdb5File);
    }















}  // end namespace frydom