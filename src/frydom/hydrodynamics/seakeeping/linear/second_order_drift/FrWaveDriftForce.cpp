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


#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"
#include "frydom/environment/FrEnvironmentInc.h"
#include "FrWaveDriftForce.h"
#include "frydom/IO/FrHDF5.h"

#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroDB.h"


namespace frydom {

    FrWaveDriftForce::FrWaveDriftForce(const std::string hdf5_file) {

        FrHDF5Reader reader;
        reader.SetFilename(hdf5_file);

        std::string mode_path("/mode_");
        std::string head_path("/heading_");

        int n_mode, n_dir;
        std::string imode_path, idir_path;

        m_sym_x = (bool)reader.ReadBool("sym_x");
        m_sym_y = (bool)reader.ReadBool("sym_y");

        m_NbModes = reader.ReadInt("n_mode");
        double NbAngles;


        char buffer [20];
        double val;

        //auto headings = std::make_shared<std::vector<double>>();
        //auto freqs = std::make_shared<std::vector<double>>();
        auto headings = std::vector<double>();
        auto freqs = std::vector<double>();
        auto omega = std::vector<double>();

        std::vector<std::vector<double>> data;
        //std::shared_ptr<std::vector<double>> coeffs;
        std::vector<double> coeffs;

        for (unsigned int imode=1; imode<=m_NbModes; ++imode) {

            sprintf(buffer, "%d", imode);
            imode_path = mode_path + buffer;

            NbAngles = reader.ReadInt(imode_path + "/n_dir");

            coeffs.clear();
            headings.clear();
            //omega.clear();

            auto table = new mathutils::LookupTable2d<>();

            for (unsigned int idir=1; idir<=NbAngles; idir++) {

                sprintf(buffer, "%d", idir);
                idir_path = imode_path + head_path + buffer;

                val = reader.ReadDouble(idir_path + "/heading");
                headings.push_back(val);

                data = reader.ReadDoubleArraySTD(idir_path + "/data");
                coeffs.insert(std::end(coeffs), std::begin(data[0]), std::end(data[0]));
            }

            data = reader.ReadDoubleArraySTD(idir_path + "/freq");
            //freqs = std::make_shared<std::vector<double>>(data[0]);
            freqs = std::vector<double>(data[0]);


            table->SetX(headings);
            table->SetY(freqs);
            table->AddData("Data", coeffs);

            auto table_unique = std::unique_ptr<mathutils::LookupTable2d<>>(table);

            m_table.push_back(std::move(table_unique));

        }
    }

    double FrWaveDriftForce::SetRelativeAngle(const double waveDir, const double heading) {

        double relative_angle;

        relative_angle = waveDir - heading;

        if (m_sym_x && relative_angle < -DBL_EPSILON) {
            relative_angle = -relative_angle;
        }

        if (m_sym_x && relative_angle - 180. > DBL_EPSILON) {
            relative_angle = 360. - relative_angle;
        }

        if (m_sym_y && relative_angle - 90. > DBL_EPSILON) {
            relative_angle = 180. - relative_angle;
        }

        relative_angle = Normalize_0_360(relative_angle);

        return relative_angle;

    }

    void FrWaveDriftForce::Initialize() {

        m_waveAmplitude = m_waveProbe->GetWaveField()->_GetWaveAmplitudes();
        FrForce::Initialize();

    }

    void FrWaveDriftForce::UpdateState() {

        //std::vector<double> drift_coeff(3); //FIXME : dynamic behaviour

        auto nbFreq = m_waveProbe->GetWaveField()->GetNbFrequencies();
        auto nbWaveDir = m_waveProbe->GetWaveField()->GetNbWaveDirections();

        auto heading = m_body->GetHeadingAngle(NWU, DEG);

        std::vector<double> cforce = {0.,0.,0.};

        auto waveDir = m_waveProbe->GetWaveField()->GetWaveDirections(DEG);
        auto w = m_waveProbe->GetEncounterWaveFrequencies();
        double wi, relative_angle;
        double dw;

        for (unsigned int idir=0; idir<nbWaveDir; ++idir) {
            dw = w[idir][1] - w[idir][0];
            for (unsigned int ifreq=0; ifreq < nbFreq; ++ifreq) {

                wi  = w[idir][ifreq];
                relative_angle = SetRelativeAngle(waveDir[idir],heading);

                cforce.at(0) += std::pow(m_waveAmplitude[idir][ifreq],2) * m_table[0]->Eval("Data", relative_angle, wi) * dw;
                cforce.at(1) += std::pow(m_waveAmplitude[idir][ifreq],2) * m_table[1]->Eval("Data", relative_angle, wi) * dw;
                cforce.at(2) += std::pow(m_waveAmplitude[idir][ifreq],2) * m_table[2]->Eval("Data", relative_angle, wi) * dw;
            }
        }

        force.x() = cforce.at(0);   //  Surge
        force.y() = cforce.at(1);   //  Sway
        force.z() = 0.;

        force = GetBody()->Dir_Body2World(force);

        moment.z() = cforce.at(2);  //  Yaw

     }







































    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    FrWaveDriftForce_::FrWaveDriftForce_(std::shared_ptr<FrHydroDB_> hdb)
        : m_hdb(hdb) {}

    void FrWaveDriftForce_::Update(double time) {

        auto waveAmplitude = m_body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

    }

    void FrWaveDriftForce_::Initialize() {
        FrForce_::Initialize();

    }

    void FrWaveDriftForce_::StepFinalize() {
        FrForce_::StepFinalize();
    }

    void FrWaveDriftForce_::SetInterpolationTable() {

        auto polarData = m_hdb->GetBody(m_body)->GetWaveDrift();



    }

}
