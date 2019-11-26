//
// Created by frongere on 19/11/2019.
//

#include "FrConfig.h"

#include "FrFileSystem.h"
#include "frydom/logging/FrEventLogger.h"


#define CONFIG_FILE_NAME ".frydom_config"


namespace frydom {


  FrConfig::FrConfig() {  // FIXME : voir si on met en place un comportement plus generique vis a vis des champs du json
    const std::string config_file = LookForConfigFile();

    if (config_file.empty()) {
//      m_json_node = json(); // empty json node...
//      return;
      m_log_folder = FrFileSystem::cwd();
    }

    // A configuration file has been found
    std::ifstream ifs(config_file);
    try {
      // Is the file correct ? (does it have a frydom_config root node ?
      m_log_folder = json::parse(ifs)["frydom_config"]["log_folder"];
    } catch (nlohmann::detail::parse_error &err) {
//      m_json_node = json();
    }

    // FIXME : d'autres choses a faire ?
  }

  std::string FrConfig::LookForConfigFile() {

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

    event_logger::info("FRyDoM", "",
                       "No configuration file \"{}\" neither in the current directory or the home directory found. Using default settings",
                       CONFIG_FILE_NAME);
    return "";

  }

  const std::string &FrConfig::GetLogFolder() const {
    return m_log_folder;
  }


}  // end namespace frydom
