//
// Created by frongere on 17/10/17.
//

#include "yaml-cpp/yaml.h"

#include "FrHydroDB.h"
#include "frydom/IO/FrHDF5.h"


namespace frydom {


//    FrRadiationIRFDB LoadIRFData(std::string yaml_file, std::string key) {
//
//        YAML::Node data = YAML::LoadFile(yaml_file);
//
//        if (data["key"]) {
//
//            auto node = data["key"];
//
//            auto nbForce = node["nbForce"].as<unsigned int>();
//            auto nbDOF = node["nbDOF"].as<unsigned int>();
//            auto nt = node["nt"].as<unsigned int>();
//            auto tf = node["tf"].as<double>();
//            auto dataFile = node["dataFile"].as<std::string>();
//
//            // Opening the datafile
//            // FIXME --> stocker les donnes IRF dans un fichier csv ?
//
//
//
//
//            // Instance of the DB
//            auto db = FrRadiationIRFDB(nbForce, nbDOF);
//
//            db.SetTime(tf, nt);
//
//
//        }




    std::shared_ptr<FrRadiationIRFDB> LoadIRFData(std::string yaml_file, std::string key) {


        YAML::Node data = YAML::LoadFile(yaml_file);


        if (!data[key]) {  // TODO: gerer les erreurs plus proprement
            throw "ERROR";
        }

        auto node = data[key];

        auto nbForce = node["nbForce"].as<unsigned int>();
        auto nbDOF = node["nbDOF"].as<unsigned int>();
        auto h5File = node["hdbFile"].as<std::string>();


        IO::FrHDF5Reader reader;
        reader.SetFilename(h5File);


        // Getting the time informations
        std::string t_key = "/body1/hydro_coeffs/radiation_damping/impulse_response_fun/t";
        auto time = reader.ReadArray(t_key);

        // For each mode, get the impulse response functions
        char mode_str [7];
        std::string root = "/body1/hydro_coeffs/radiation_damping/impulse_response_fun/components/K/";
        std::string full;
        for (auto iforce=0; iforce<nbForce; ++iforce) {
            for (auto idof=0; idof<nbDOF; ++idof) {
                std::sprintf(mode_str, "%u_%u", iforce+1, idof+1);

                full = root + mode_str;

                std::cout << reader.ReadArray(full);
            }
        }



//        return std::shared_ptr<FrRadiationIRFDB>();
    }


}  // end namespace frydom