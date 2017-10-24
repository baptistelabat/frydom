//
// Created by frongere on 17/10/17.
//

#include <boost/filesystem.hpp>

#include "yaml-cpp/yaml.h"

#include "FrHydroDB.h"
#include "frydom/IO/FrHDF5.h"


namespace frydom {


    std::shared_ptr<FrRadiationIRFDB> LoadIRFData(std::string yaml_file, std::string key) {

        YAML::Node data = YAML::LoadFile(yaml_file);

        if (!data[key]) {  // TODO: gerer les erreurs plus proprement
            throw "ERROR";
        }

        auto node = data[key];
        // TODO: mettre dans un try !!!!
        auto nbForce = node["nbForce"].as<unsigned int>();
        auto nbDOF = node["nbDOF"].as<unsigned int>();
        auto h5File = node["hdbFile"].as<std::string>();

        // making the filepath relative to the yaml_file
        // Getting the root of the filepath to the yaml file
        boost::filesystem::path yaml_path(yaml_file);
        auto h5_path = yaml_path.parent_path();
        h5_path /= h5File;

        IO::FrHDF5Reader reader;
        reader.SetFilename(h5_path.native());


        auto IRFDB = std::make_shared<FrRadiationIRFDB>(nbForce, nbDOF);

        // Getting the time informations
        std::string t_key = "/body1/hydro_coeffs/radiation_damping/impulse_response_fun/t";
        auto time = reader.ReadArray(t_key);
        auto nt = time.size();

        IRFDB->SetTime(time(nt-1), (uint)nt);

        // For each mode, get the impulse response functions
        char mode_str [40];
        std::string root = "/body1/hydro_coeffs/radiation_damping/impulse_response_fun/components/K/";
        std::string full;
        for (unsigned int iforce=0; iforce<nbForce; ++iforce) {
            for (unsigned int idof=0; idof<nbDOF; ++idof) {
                std::sprintf(mode_str, "%u_%u", iforce+1, idof+1);

                full = root + mode_str;

                auto Kij = reader.ReadArray(full);

//                std::cout << Kij << "\n\n\n";

                // TODO: passer Kij en std::vector
                std::vector<double> Kij_v;
                for (int i=0; i<nt; ++i) {
                    Kij_v.push_back(Kij(i, 1));
                }

                IRFDB->SetKernel(iforce, idof, Kij_v);
            }
        }

        return IRFDB;
    }


}  // end namespace frydom