//
// Created by frongere on 19/11/2019.
//

#include "FrConfigFile.h"

#include "FrFileSystem.h"
#include "frydom/logging/FrEventLogger.h"


#define CONFIG_FILE_NAME ".frydom_config"


namespace frydom {


  FrConfigFile::FrConfigFile() {
    const std::string config_file = SearchConfigFile();
//    if (!TestConfigFile(config_file)) {
//      event_logger::error("FRyDoM", "", "Configuration file found is not correct.");
//      return;
//    }

    std::ifstream ifs(config_file);

    m_json_node = json::parse(ifs)["frydom_config"];



  }

//  int FrConfigFile::TestConfigFile() {
//
//  }

  const std::string FrConfigFile::SearchConfigFile() {

    // Looking for a local frydom configuration file
    std::string config_file = FrFileSystem::join({FrFileSystem::cwd(), CONFIG_FILE_NAME});
    if (FrFileSystem::exists(config_file)) {
      return config_file;
    }

    // Looking for a session wide frydom configuration file
    config_file = FrFileSystem::join({FrFileSystem::get_home(), CONFIG_FILE_NAME});
    if (FrFileSystem::exists(config_file)) {
      return config_file;
    }

    event_logger::info("FRyDoM", "", "No configuration file {} found. Using default settings", CONFIG_FILE_NAME);


  }

  const std::string &FrConfigFile::GetLogFolder() const {
    return "";
  }


}  // end namespace frydom
