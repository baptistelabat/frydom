//
// Created by frongere on 17/10/17.
//

#include "FrHydroDB.h"


namespace frydom {


    FrRadiationIRFDB LoadIRFData(std::string yaml_file, std::string key) {

        YAML::Node data = YAML::LoadFile(yaml_file);

        if (data["key"]) {

            auto node = data["key"];

            auto nbForce = node["nbForce"].as<unsigned int>();
            auto nbDOF = node["nbForce"].as<unsigned int>();
            auto nt = node["nt"].as<unsigned int>();
            auto tf = node["tf"].as<double>();
            auto dataFile = node["dataFile"].as<std::string>();

            // Opening the datafile
            // FIXME --> pour le moment, on stocke les donnees de IRF




            // Instance of the DB
            auto db = FrRadiationIRFDB(nbForce, nbDOF);

            db.SetTime(tf, nt);


        }




    }
}  // end namespace frydom