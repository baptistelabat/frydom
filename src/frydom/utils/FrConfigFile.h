//
// Created by frongere on 19/11/2019.
//

#ifndef FRYDOM_FRCONFIGFILE_H
#define FRYDOM_FRCONFIGFILE_H

#include <string>

#include <nlohmann/json.hpp>





using json = nlohmann::json;


namespace frydom {

  class FrConfigFile {

   public:

    FrConfigFile();

    const std::string &GetLogFolder() const;



   private:
    static const std::string SearchConfigFile();

//    static int TestConfigFile(const std::string &config_file);


   private:
    json m_json_node;

  };

}  // end namespace frydom



#endif //FRYDOM_FRCONFIGFILE_H
