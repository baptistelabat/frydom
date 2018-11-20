//
// Created by frongere on 11/01/18.
//

#include "FrHydroDBLoader.h"
#include "FrHydroDB.h"
#include "frydom/IO/FrHDF5.h"


namespace frydom {

    FrHydroDB LoadHDB5(std::string hdb5_file) {

        FrHDF5Reader reader;
        reader.SetFilename(hdb5_file);

        // Hydrodynamics Database creation
        FrHydroDB HDB;

        auto GravityAcc = reader.ReadDouble("/GravityAcc");
        HDB.SetGravityAcc(GravityAcc);

        auto WaterDensity = reader.ReadDouble("/WaterDensity");
        HDB.SetWaterDensity(WaterDensity);

        auto NormalizationLength = reader.ReadDouble("/NormalizationLength");
        HDB.SetNormalizationLength(NormalizationLength);

        auto WaterDepth = reader.ReadDouble("/WaterDepth");
        HDB.SetWaterDepth(WaterDepth);

        auto NbBodies = reader.ReadInt("/NbBody");


        std::string discretization_path = "/Discretizations";

        // Reading frequency discretization
        std::string frequency_discretization_path = discretization_path + "/Frequency";
        auto NbFreq = reader.ReadInt(frequency_discretization_path + "/NbFrequencies");
        auto MinFreq = reader.ReadDouble(frequency_discretization_path + "/MinFrequency");
        auto MaxFreq = reader.ReadDouble(frequency_discretization_path + "/MaxFrequency");
        HDB.SetFrequencyDiscretization(MinFreq, MaxFreq, (uint)NbFreq);

        // Reading wave propagation direction discretization
        std::string wave_direction_discretization_path = discretization_path + "/WaveDirections";
        auto NbWaveDir = reader.ReadInt(wave_direction_discretization_path + "/NbWaveDirections");
        auto MinWaveDir = reader.ReadDouble(wave_direction_discretization_path + "/MinAngle");
        auto MaxWaveDir = reader.ReadDouble(wave_direction_discretization_path + "/MaxAngle");
        HDB.SetWaveDirectionDiscretization(MinWaveDir, MaxWaveDir, (uint)NbWaveDir);

        // Reading wave propagation direction discretization
        std::string time_discretization_path = discretization_path + "/Time";
        auto NbTimeSample = reader.ReadInt(time_discretization_path + "/NbTimeSample");
        auto FinalTime = reader.ReadDouble(time_discretization_path + "/FinalTime");
        HDB.SetTimeDiscretization(FinalTime, (uint)NbTimeSample);

        // Getting data from body
        std::string body_path("/Bodies/Body_");
        std::string body_i_path, mode_path, imode_path;
        char buffer [20];
        for (unsigned int ibody=0; ibody<NbBodies; ++ibody) {

            sprintf(buffer, "%d", ibody);
            body_i_path = body_path + buffer;

            auto BodyName = reader.ReadString(body_i_path + "/BodyName");
            auto body = HDB.NewBody(BodyName);

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
            auto body = HDB.GetBody(ibody);

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

                body_motion = HDB.GetBody(ibody_motion);

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

        return HDB;

    }
















//
//    //////////////   REFACTORING ---------------------->>>>>>>>>>>>>>>>>>>>>
//    FrHydroDB_ LoadHDB5_(std::string hdb5_file) {
//
//        FrHDF5Reader reader;
//        reader.SetFilename(hdb5_file);
//
//        // Hydrodynamics Database creation
//        FrHydroDB_ HDB;
//
//        auto GravityAcc = reader.ReadDouble("/GravityAcc");
//        HDB.SetGravityAcc(GravityAcc);
//
//        auto WaterDensity = reader.ReadDouble("/WaterDensity");
//        HDB.SetWaterDensity(WaterDensity);
//
//        auto NormalizationLength = reader.ReadDouble("/NormalizationLength");
//        HDB.SetNormalizationLength(NormalizationLength);
//
//        auto WaterDepth = reader.ReadDouble("/WaterDepth");
//        HDB.SetWaterDepth(WaterDepth);
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
//        HDB.SetFrequencyDiscretization(MinFreq, MaxFreq, (uint)NbFreq);
//
//        // Reading wave propagation direction discretization
//        std::string wave_direction_discretization_path = discretization_path + "/WaveDirections";
//        auto NbWaveDir = reader.ReadInt(wave_direction_discretization_path + "/NbWaveDirections");
//        auto MinWaveDir = reader.ReadDouble(wave_direction_discretization_path + "/MinAngle");
//        auto MaxWaveDir = reader.ReadDouble(wave_direction_discretization_path + "/MaxAngle");
//        HDB.SetWaveDirectionDiscretization(MinWaveDir, MaxWaveDir, (uint)NbWaveDir);
//
//        // Reading wave propagation direction discretization
//        std::string time_discretization_path = discretization_path + "/Time";
//        auto NbTimeSample = reader.ReadInt(time_discretization_path + "/NbTimeSample");
//        auto FinalTime = reader.ReadDouble(time_discretization_path + "/FinalTime");
//        HDB.SetTimeDiscretization(FinalTime, (uint)NbTimeSample);
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
//            auto body = HDB.NewBody(BodyName);
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
//            auto body = HDB.GetBody(ibody);
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
//                body_motion = HDB.GetBody(ibody_motion);
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
//
//        return HDB;
//
//    }



}  // end namespace frydom