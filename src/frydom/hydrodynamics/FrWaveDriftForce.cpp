//
// Created by camille on 14/12/17.
//

#include "frydom/environment/waves/FrWaveField.h"
#include "FrWaveDriftForce.h"
#include "frydom/IO/FrHDF5.h"

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

    void FrWaveDriftForce::SetCmplxElevation() {

        auto x = m_waveProbe->GetX();
        auto y = m_waveProbe->GetY();

        m_CmplxElevation = m_waveProbe->GetWaveField()->GetCmplxElevation(x, y, true);

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

        return relative_angle;

    }

    void FrWaveDriftForce::UpdateState() {

        //std::vector<double> drift_coeff(3); //FIXME : dynamic behaviour

        auto nbFreq = m_waveProbe->GetWaveField()->GetNbFrequencies();
        auto nbWaveDir = m_waveProbe->GetWaveField()->GetNbWaveDirections();

        auto heading = m_body->GetHeadingAngle(NED, DEG);

        auto emjwt = m_waveProbe->GetWaveField()->GetTimeCoeffs();

        std::vector<std::complex<double>> cforce = {0.,0.,0.};

        auto waveDir = m_waveProbe->GetWaveField()->GetWaveDirections(DEG);
        auto w = m_waveProbe->GetWaveField()->GetWaveFrequencies(RADS);
        double wi, relative_angle;

        for (unsigned int idir=0; idir<nbWaveDir; ++idir) {
            for (unsigned int ifreq=0; ifreq < nbFreq; ++ifreq) {

                wi  = w[ifreq];
                relative_angle = SetRelativeAngle(waveDir[idir],heading);

                cforce.at(0) += m_CmplxElevation[idir][ifreq] * m_table[0]->Eval("Data", relative_angle, wi) * emjwt[ifreq];
                cforce.at(1) += m_CmplxElevation[idir][ifreq] * m_table[1]->Eval("Data", relative_angle, wi) * emjwt[ifreq];
                cforce.at(2) += m_CmplxElevation[idir][ifreq] * m_table[2]->Eval("Data", relative_angle, wi) * emjwt[ifreq];

            }
        }

        force.x() = std::real(cforce.at(0));   //  Surge
        force.y() = std::real(cforce.at(1));   //  Sway

        force = GetBody()->Dir_Body2World(force);

        moment.z() = std::real(cforce.at(2));  //  Yaw

     }

}