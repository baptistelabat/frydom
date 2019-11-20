//
// Created by frongere on 19/11/2019.
//

#ifndef FRYDOM_FRCONFIG_H
#define FRYDOM_FRCONFIG_H

#include <string>

#include <nlohmann/json.hpp>


using json = nlohmann::json;


// TODO : avoir une classe de ppte disant qsi c'est set ou pas...

namespace frydom {

  namespace internal {
    template<typename T>
    class ConfigValue;
  }

  class FrConfig {

   public:

    FrConfig();

    const std::string &GetLogFolder() const;


   private:
    static std::string LookForConfigFile();

//    static int TestConfigFile(const std::string &config_file);


   private:
    json m_json_node;

  };


  namespace internal {
    template<typename T>
    class ConfigValue {
     public:

      ConfigValue() {

      }

      ConfigValue(const T val, bool) {

      }

      const T &Get() const;

     private:
      bool m_was_set;
      T m_val;

    };
  }

}  // end namespace frydom



#endif //FRYDOM_FRCONFIG_H
