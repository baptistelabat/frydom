//
// Created by camille on 14/12/17.
//

#include "frydom/environment/waves/FrWaveField.h"
#include "FrWaveDriftForce.h"
#include "frydom/IO/FrHDF5.h"

namespace frydom {

    FrWaveDriftForce::FrWaveDriftForce(const std::string hdf5_file) {

        IO::FrHDF5Reader reader;
        reader.SetFilename(hdf5_file);

        std::string mode_path("/mode_");
        std::string head_path("/heading_");

        int n_mode, n_dir;
        std::string imode_path, idir_path;

        auto sym_x = reader.ReadBool("sym_x");
        auto sym_y = reader.ReadBool("sym_y");

        m_NbModes = reader.ReadInt("n_mode");
        double NbAngles;

        char buffer [20];
        double val;

        auto headings = std::make_shared<std::vector<double>>();
        auto freqs = std::make_shared<std::vector<double>>();

        std::vector<std::vector<double>> data;
        std::shared_ptr<std::vector<double>> coeffs;

        for (unsigned int imode=1; imode<=m_NbModes; ++imode) {

            sprintf(buffer, "%d", imode);
            imode_path = mode_path + buffer;

            NbAngles = reader.ReadInt(imode_path + "/n_dir");

            for (unsigned int idir=1; idir<=NbAngles; idir++) {

                sprintf(buffer, "%d", idir);
                idir_path = imode_path + head_path + buffer;

                val = reader.ReadDouble(idir_path + "/heading");
                headings->push_back(val);

                data = reader.ReadDoubleArraySTD(idir_path + "/data");
                //coeffs->push_back(data[0]);
                coeffs->insert(std::end(*coeffs), std::begin(data[0]), std::end(data[0]));
            }

            data = reader.ReadDoubleArraySTD(idir_path + "/freq");
            freqs = std::make_shared<std::vector<double>>(data[0]);

            m_table[imode].SetX(*headings);
            m_table[imode].SetY(*freqs);
            m_table[imode].AddData("Data", *coeffs);

        }


    }

    /**
    void FrWaveDriftForce::BuildDriftCoefficientInterpolators() {

        m_headInterpolators.clear();
        m_headInterpolators.reserve(m_NbModes);

        Eigen::MatrixXcd data;
        auto angles = GetHeadings();

        auto interpolators = std::vector<Interp1dLinear<double, double>>();
        interpolators.reserve(NbFreq);

        for (unsigned int imode=0; imode<m_NbModes; imode++) {

            interpolators.clear();

            for (unsigned int ifreq=0; ifreq<NbFreq; ifreq++) {

                auto coeffs = std::make_shared<std::vector<double>>();
                coeffs->reserve(m_NbAngles[imode]);

                for (unsigned int iangle=0; iangle<m_NbAngles[imode]; ++iangle) {
                    data = GetCoefficient(iangle);
                    coeffs->push_back(data(imode, ifreq));
                }

                auto interpolator = Interp1dLinear<double, double>();
                interpolator.Initialize(angles, coeffs);
                interpolators.push_back(interpolator);

            }
            m_headInterpolators.push_back(interpolators);

        }


    }
**/

    void FrWaveDriftForce::SetCmplxElevation() {

        auto x = m_waveProbe->GetX();
        auto y = m_waveProbe->GetY();

        m_CmplxElevation = m_waveProbe->GetWaveField()->GetCmplxElevation(x, y, true);

    }

    void FrWaveDriftForce::UpdateState() {

        //std::vector<double> drift_coeff(3); //FIXME : dynamic behaviour

        auto nbFreq = m_waveProbe->GetWaveField()->GetNbFrequencies();
        auto nbWaveDir = m_waveProbe->GetWaveField()->GetNbWaveDirections();

        auto heading = m_body->GetHeadingAngle(NED);

        auto emjwt = m_waveProbe->GetWaveField()->GetTimeCoeffs();

        std::vector<std::complex<double>> cforce = {0.,0.,0.};

        auto waveDir = m_waveProbe->GetWaveField()->GetWaveDirections(RAD);
        auto w = m_waveProbe->GetWaveField()->GetWaveFrequencies(RADS);
        double wi, relative_angle;

        for (unsigned int idir=0; idir<nbWaveDir; ++idir) {
            for (unsigned int ifreq=0; ifreq < nbFreq; ++ifreq) {

                wi  =w[ifreq];
                relative_angle = heading - waveDir[idir];

                //drift_coeff = {m_table.Eval("Surge", wi, relative_angle),
                //               m_table.Eval("Sway", wi, relative_angle),
               //               m_table.Eval("Yaw", wi, relative_angle)};

                //cforce += m_CmplxElevation[idir][ifreq] .* drift_coeff * emjwt[ifreq];

                cforce.at(0) += m_CmplxElevation[idir][ifreq] * m_table[0].Eval("Data", wi, relative_angle) * emjwt[ifreq];
                cforce.at(1) += m_CmplxElevation[idir][ifreq] * m_table[1].Eval("Data", wi, relative_angle) * emjwt[ifreq];
                cforce.at(2) += m_CmplxElevation[idir][ifreq] * m_table[2].Eval("Data", wi, relative_angle) * emjwt[ifreq];

            }
        }
        force.x() = std::real(cforce.at(0));   //  Surge
        force.y() = std::real(cforce.at(1));   //  Sway
        moment.z() = std::real(cforce.at(2));  //  Yaw
     }




}