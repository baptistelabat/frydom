//
// Created by frongere on 17/10/17.
//

//#include <boost/filesystem.hpp>

#include "yaml-cpp/yaml.h"

#include "FrHydroDB.h"
#include "frydom/misc/FrLinspace.h"
#include "frydom/misc/FrInterp1d.h"
#include "frydom/IO/FrHDF5.h"


namespace frydom {

    FrHydroDB LoadHDB5(std::string hdb5_file) {

        IO::FrHDF5Reader reader;
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
                diffractionCoeffs = diffraction_realCoeffs + J * diffraction_imagCoeffs;
                body->SetDiffraction(iwave_dir, diffractionCoeffs);

                // Reading Froude-Krylov coefficients
                fk_wave_dir_path = froude_kylov_path + buffer;

                auto fk_realCoeffs = reader.ReadDoubleArray(fk_wave_dir_path + "/RealCoeffs");
                auto fk_imagCoeffs = reader.ReadDoubleArray(fk_wave_dir_path + "/ImagCoeffs");

                Eigen::MatrixXcd froudeKrylovCoeffs;
                froudeKrylovCoeffs = fk_realCoeffs + J * fk_imagCoeffs;
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

            body->Finalize();

        }  // end for ibody (force)

        return HDB;

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

    unsigned int FrBEMBody::GetNbFrequencies() const {
        return m_HDB->GetNbFrequencies();
    }

    unsigned int FrBEMBody::GetNbWaveDirections() const {
        return m_HDB->GetNbWaveDirections();
    }

    void FrBEMBody::BuildInterpolators() {
        std::cout << "Generating HDB interpolators" << std::endl;
        // Wave Exxcitation interpolators
        BuildWaveExcitationInterpolators();
        // Radiation interpolators
//        BuildRadiationInterpolators(); // FIXME: c'est plutot a l'echelle de la HDB...
    }

    void FrBEMBody::BuildWaveExcitationInterpolators() {

        auto nbWaveDirections = GetNbWaveDirections();
        auto nbForceModes = GetNbForceMode();

//        auto interp = FrInterp1dLinear<double, std::complex<double>>();
//        std::vector<std::vector<FrInterp1dLinear>> waveDirectionsInterpolators; // Interpolate first wave directions then force modes against wave frequencies
//        waveDirectionsInterpolators.reserve(nbWaveDirections);
//
////        auto frequencies = Get;
//
//        for (unsigned int iWaveDir=0; iWaveDir<nbWaveDirections; ++iWaveDir) {
//
//            for (unsigned int iMode=0; iMode<nbForceModes; ++iMode) {
//
//                auto frequencyInterpolator = FrInterp1dLinear();
//
//
//
//
//            }
//
//        }





    }

//    void FrBEMBody::BuildRadiationInterpolators() {
//        // TODO
//    }


    std::vector<double> FrDiscretization1D::GetVector() const {
        return linspace<double>(m_xmin, m_xmax, m_nx);
    }
}  // end namespace frydom