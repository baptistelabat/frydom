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

        // Reading frequency discretization
        auto NbFreq = reader.ReadInt("/discretizations/frequency/nbFrequencies");
        auto MinFreq = reader.ReadDouble("/discretizations/frequency/minFrequency");
        auto MaxFreq = reader.ReadDouble("/discretizations/frequency/maxFrequency");
        auto omega = linspace<double>(MinFreq, MaxFreq, (uint)NbFreq);

        // Reading wave propagation direction discretization
        auto NbWaveDir = reader.ReadInt("/discretizations/wave_directions/nbWaveDirections");
        auto MinWaveDir = reader.ReadDouble("/discretizations/wave_directions/minAngle");
        auto MaxWaveDir = reader.ReadDouble("/discretizations/wave_directions/maxAngle");
        auto WaveDirections = linspace<double>(MinWaveDir, MaxWaveDir, (uint)NbWaveDir);

        // Reading wave propagation direction discretization
        auto NbTimeSample = reader.ReadInt("/discretizations/time/nbTimeSample");
        auto FinalTime = reader.ReadDouble("/discretizations/time/finalTime");
        auto time = linspace<double>(0., FinalTime, (uint)NbTimeSample);

        // Getting data from body
        std::string body_path("/Body_");
        std::string ibody_path, mode_path, imode_path;
        char buffer [20];
        for (unsigned int ibody=0; ibody<NbBodies; ++ibody) {

            sprintf(buffer, "%d", ibody);
            ibody_path = body_path + buffer;

            auto BodyName = reader.ReadString(ibody_path + "/BodyName");

            auto BodyPosition = reader.ReadDoubleArray(ibody_path + "/BodyPosition");

            auto ID = reader.ReadInt(ibody_path + "/ID");

            FrBEMBody body;

            auto nbForceModes = reader.ReadInt(ibody_path + "/nbForceModes");

            mode_path = ibody_path + "/force_modes/mode_";
            std::string mode_type;
            Eigen::Vector3d direction, point;
            for (unsigned int iforce=0; iforce<nbForceModes; ++iforce) {
                sprintf(buffer, "%d", iforce);
                imode_path = mode_path + buffer;

                // Building the force mode
                FrBEMForceMode mode;

                mode_type = reader.ReadString(imode_path + "/type");
                direction = reader.ReadDoubleArray(imode_path + "/direction");

                mode.SetDirection(direction);

                if (mode_type == "moment") {
                    point = reader.ReadDoubleArray(imode_path + "/application_point");
                    mode.SetTypeMOMENT();
                    mode.SetPoint(point);
                } else {
                    mode.SetTypeFORCE();
                }

                // Adding the mode to the BEMBody
                body.AddForceMode(mode);

            }  // end for iforce


            auto nbMotionModes = reader.ReadInt(ibody_path + "/nbMotionModes");

            mode_path = ibody_path + "/motion_modes/mode_";
            for (unsigned int imotion=0; imotion<nbForceModes; ++imotion) {
                sprintf(buffer, "%d", imotion);
                imode_path = mode_path + buffer;

                // Building the force mode
                FrBEMMotionMode mode;

                mode_type = reader.ReadString(imode_path + "/type");
                direction = reader.ReadDoubleArray(imode_path + "/direction");

                mode.SetDirection(direction);

                if (mode_type == "rotation") {
                    point = reader.ReadDoubleArray(imode_path + "/rotation_point");
                    mode.SetTypeROTATION();
                    mode.SetPoint(point);
                } else {
                    mode.SetTypeTRANSLATION();
                }

                // Adding the mode to the BEMBody
                body.AddMotionMode(mode);

            }  // end for imotion

            // Reading the mesh
            std::string ibody_mesh_path = ibody_path + "/mesh";
            auto nbVertices = reader.ReadInt(ibody_mesh_path + "/nbVertices");
            auto vertices = reader.ReadDoubleArray(ibody_mesh_path + "/vertices");

            auto nbFaces = reader.ReadInt(ibody_mesh_path + "/nbFaces");
            auto faces = reader.ReadIntArray(ibody_mesh_path + "/faces");




            Bodies.push_back(body);


        }  // end for ibody

        // READING THE HYDRODYNAMIC COEFFICIENTS
        for (unsigned int ibody=0; ibody<NbBodies; ++ibody) {

            sprintf(buffer, "%d", ibody);
            ibody_path = body_path + buffer;

            auto body = Bodies[ibody];

            // Reading the excitation hydrodynamic coefficients
            auto diffraction_path = ibody_path + "/excitation/diffraction";
            auto froude_kylov_path = ibody_path + "/excitation/froude_krylov";
            std::string diffraction_wave_dir_path, fk_wave_dir_path;
            for (unsigned int iwave_dir=0; iwave_dir<NbWaveDir; ++iwave_dir) {
                sprintf(buffer, "/angle_%d", iwave_dir);

                // Reading diffraction coefficients
                diffraction_wave_dir_path = diffraction_path + buffer;

                auto diffraction_realCoeffs = reader.ReadDoubleArray(diffraction_wave_dir_path + "/realCoeffs");
                auto diffraction_imagCoeffs = reader.ReadDoubleArray(diffraction_wave_dir_path + "/imagCoeffs");

                Eigen::MatrixXcd diffractionCoeffs;
                diffractionCoeffs = diffraction_realCoeffs + J * diffraction_imagCoeffs;

                // Reading Froude-Krylov coefficients
                fk_wave_dir_path = froude_kylov_path + buffer;

                auto fk_realCoeffs = reader.ReadDoubleArray(fk_wave_dir_path + "/realCoeffs");
                auto fk_imagCoeffs = reader.ReadDoubleArray(fk_wave_dir_path + "/imagCoeffs");

                Eigen::MatrixXcd froudeKrylovCoeffs;
                froudeKrylovCoeffs = fk_realCoeffs + J * fk_imagCoeffs;

                // TODO: mettre dans la database ExcitationDB

            }

            // Reading the radiation coefficients
            auto radiation_path = ibody_path + "/radiation";
            FrBEMBody body_motion;
            for (unsigned int ibody_motion=0; ibody_motion<NbBodies; ++ibody_motion) {
                sprintf(buffer, "/Body_%d_motion", ibody_motion);

                body_motion = Bodies[ibody];

                // Added mass
                auto ibody_added_mass_path = radiation_path + buffer + "/added_mass";
                auto ibody_wave_damping_path = radiation_path + buffer + "/wave_damping";
                auto ibody_infinite_added_mass_path = radiation_path + buffer + "/infinite_added_mass";
                auto ibody_impulse_response_function_path = radiation_path + buffer + "/impulse_response_function";
                for (unsigned int imotion=0; imotion<body_motion.GetNbMotionMode(); ++imotion) {
                    sprintf(buffer, "/Body_%d_DOF_%d", ibody_motion, imotion);

                    auto added_mass = reader.ReadDoubleArray(ibody_added_mass_path + buffer);

                    auto wave_damping = reader.ReadDoubleArray(ibody_wave_damping_path + buffer);

                    // TODO: ajouter l'ecriture des CMinf et des IRF dans le writer python

//                    auto infinite_added_mass = reader.ReadDoubleArray(ibody_infinite_added_mass_path + buffer);

//                    auto impulse_response_function = reader.ReadDoubleArray(ibody_impulse_response_function_path + buffer);

                    // TODO: charger dans les bases de données

                }





            }



        }  // end for ibody



    }


}  // end namespace frydom