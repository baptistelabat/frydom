//
// Created by frongere on 17/10/17.
//

//#include <boost/filesystem.hpp>

#include "yaml-cpp/yaml.h"

#include "FrHydroDB.h"
#include "frydom/IO/FrHDF5.h"


namespace frydom {

    FrHydroDB LoadHDB5(std::string hdb5_file) {

        IO::FrHDF5Reader reader;
        reader.SetFilename(hdb5_file);

        auto GravityAcc = reader.ReadDouble("/GravityAcc");

        auto WaterDensity = reader.ReadDouble("/WaterDensity");

        auto NormalizationLength = reader.ReadDouble("/NormalizationLength");

        auto WaterDepth = reader.ReadDouble("/WaterDepth");

        auto NbBodies = reader.ReadInt("/NbBody");

        std::vector<FrBEMBody> Bodies;
        Bodies.reserve((uint)NbBodies);

        std::string discretization_path = "/Discretizations";

        // Reading frequency discretization
        std::string frequency_discretization_path = discretization_path + "/Frequency";
        auto NbFreq = reader.ReadInt(frequency_discretization_path + "/NbFrequencies");
        auto MinFreq = reader.ReadDouble(frequency_discretization_path + "/MinFrequency");
        auto MaxFreq = reader.ReadDouble(frequency_discretization_path + "/MaxFrequency");
        auto omega = linspace<double>(MinFreq, MaxFreq, (uint)NbFreq);

        // Reading wave propagation direction discretization
        std::string wave_direction_discretization_path = discretization_path + "/WaveDirections";
        auto NbWaveDir = reader.ReadInt(wave_direction_discretization_path + "/NbWaveDirections");
        auto MinWaveDir = reader.ReadDouble(wave_direction_discretization_path + "/MinAngle");
        auto MaxWaveDir = reader.ReadDouble(wave_direction_discretization_path + "/MaxAngle");
        auto WaveDirections = linspace<double>(MinWaveDir, MaxWaveDir, (uint)NbWaveDir);

        // Reading wave propagation direction discretization
        std::string time_discretization_path = discretization_path + "/Time";
        auto NbTimeSample = reader.ReadInt(time_discretization_path + "/NbTimeSample");
        auto FinalTime = reader.ReadDouble(time_discretization_path + "/FinalTime");
        auto time = linspace<double>(0., FinalTime, (uint)NbTimeSample);

        // Getting data from body
        std::string body_path("/Bodies/Body_");
        std::string body_i_path, mode_path, imode_path;
        char buffer [20];
        for (unsigned int ibody=0; ibody<NbBodies; ++ibody) {

            sprintf(buffer, "%d", ibody);
            body_i_path = body_path + buffer;

            auto BodyName = reader.ReadString(body_i_path + "/BodyName");

            auto BodyPosition = reader.ReadDoubleArray(body_i_path + "/BodyPosition");

            auto ID = reader.ReadInt(body_i_path + "/ID");

            FrBEMBody body;

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
                body.AddForceMode(mode);

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
                body.AddMotionMode(mode);

            }  // end for imotion

            // Reading the mesh
            std::string ibody_mesh_path = body_i_path + "/Mesh";
            auto nbVertices = reader.ReadInt(ibody_mesh_path + "/NbVertices");
            auto vertices = reader.ReadDoubleArray(ibody_mesh_path + "/Vertices");

            auto nbFaces = reader.ReadInt(ibody_mesh_path + "/NbFaces");
            auto faces = reader.ReadIntArray(ibody_mesh_path + "/Faces");

            Bodies.push_back(body);


        }  // end for ibody

        // READING THE HYDRODYNAMIC COEFFICIENTS
        for (unsigned int ibody=0; ibody<NbBodies; ++ibody) {

            sprintf(buffer, "%d", ibody);
            body_i_path = body_path + buffer;

            auto body = Bodies[ibody];

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
                diffractionCoeffs = diffraction_realCoeffs + J * diffraction_imagCoeffs;

                // Reading Froude-Krylov coefficients
                fk_wave_dir_path = froude_kylov_path + buffer;

                auto fk_realCoeffs = reader.ReadDoubleArray(fk_wave_dir_path + "/RealCoeffs");
                auto fk_imagCoeffs = reader.ReadDoubleArray(fk_wave_dir_path + "/ImagCoeffs");

                Eigen::MatrixXcd froudeKrylovCoeffs;
                froudeKrylovCoeffs = fk_realCoeffs + J * fk_imagCoeffs;

            }

            // Reading the radiation coefficients
            auto radiation_path = body_i_path + "/Radiation";
            FrBEMBody body_motion;
            for (unsigned int ibody_motion=0; ibody_motion<NbBodies; ++ibody_motion) {
                sprintf(buffer, "/BodyMotion_%d", ibody_motion);

                body_motion = Bodies[ibody];

                auto body_i_added_mass_path = radiation_path + buffer + "/AddedMass";
                auto body_i_radiation_damping_path = radiation_path + buffer + "/RadiationDamping";
                auto body_i_infinite_added_mass_path = radiation_path + buffer + "/InfiniteAddedMass";
                auto body_i_impulse_response_function_path = radiation_path + buffer + "/ImpulseResponseFunction";
                for (unsigned int imotion=0; imotion<body_motion.GetNbMotionMode(); ++imotion) {
                    sprintf(buffer, "/DOF_%d", imotion);

                    auto added_mass = reader.ReadDoubleArray(body_i_added_mass_path + buffer);

                    auto wave_damping = reader.ReadDoubleArray(body_i_radiation_damping_path + buffer);

                    auto infinite_added_mass = reader.ReadDoubleArray(body_i_infinite_added_mass_path);

                    auto impulse_response_function = reader.ReadDoubleArray(body_i_impulse_response_function_path + buffer);

                }  // end for imotion
            }  // end ibody_motion
        }  // end for ibody (force)


    }


    void FrBEMBody::Initialize() {
        assert(!m_ForceModes.empty() && !m_MotionModes.empty());

        auto nbForce = GetNbForceMode();

        // Allocating arrays for excitations
        auto NbWaveDir = m_HDB->GetNbWaveDirections();
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
        m_InfiniteAddedMass.reserve(NbBodies);
        m_AddedMass.reserve(NbBodies);
        m_RadiationDamping.reserve(NbBodies);
        m_ImpulseResponseFunction.reserve(NbBodies);

        auto NbTime = m_HDB->GetNbTimeSamples();
        for (unsigned int ibody=0; ibody<NbBodies; ++ibody) {
            auto body = m_HDB->GetBody(ibody);
            auto NbMotion = body->GetNbMotionMode();

            Eigen::MatrixXd InfAddedMassMat(nbForce, NbMotion);
            m_InfiniteAddedMass.push_back(InfAddedMassMat);

            m_AddedMass[ibody].reserve(NbMotion);
            m_RadiationDamping[ibody].reserve(NbMotion);
            m_ImpulseResponseFunction[ibody].reserve(NbMotion);

            for (unsigned int idof=0; idof<NbMotion; ++idof) {
                Eigen::MatrixXd mat(nbForce, NbFreq);
                m_AddedMass[ibody].push_back(mat);
                m_RadiationDamping[ibody].push_back(mat);

                Eigen::MatrixXd matTime(nbForce, NbTime);
                m_ImpulseResponseFunction[ibody].push_back(matTime);
            }


        }




    }

    void FrBEMBody::ComputeExcitation() {
        for (unsigned int iangle=0; iangle<m_HDB->GetNbWaveDirections(); ++iangle) {
            m_Excitation[iangle] = m_Diffraction[iangle] + m_FroudeKrylov[iangle];
        }
    }
}  // end namespace frydom